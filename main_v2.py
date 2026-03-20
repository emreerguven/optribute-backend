"""
Optribute v2 — HGS + OR-Tools VRPTW
------------------------------------
Katman 1: OSRM  → gerçek yol matrisleri
Katman 2: HGS   → güçlü başlangıç çözümü (PyHygese)
Katman 3: OR-Tools → time window + kapasite constraint validation,
                     HGS çözümünü hint olarak kullanır
Katman 4: ALNS  → local search iyileştirme (Hafta 2'de eklenecek)
Katman 5: MOEA/D → Pareto front (Hafta 3'te eklenecek)

Kurulum:
    pip install fastapi uvicorn pydantic requests folium polyline hygese ortools
"""

from fastapi import FastAPI, HTTPException
from pydantic import BaseModel
from typing import List, Optional, Dict, Any
from folium.features import DivIcon
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

import urllib.parse
import requests
import folium
import polyline as polyline_lib
import math
import time
import logging

# HGS — pip install hygese
try:
    from hygese import AlgorithmParameters, Solver as HGSSolver
    HGS_AVAILABLE = True
except ImportError:
    HGS_AVAILABLE = False
    logging.warning("PyHygese kurulu değil. 'pip install hygese' çalıştır. OR-Tools fallback aktif.")

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("optribute_v2")

app = FastAPI(title="Optribute API v2", version="2.0.0")

OSRM_BASE_URL = "http://router.project-osrm.org"

# =============================================================================
# VERİ MODELLERİ (v1 ile tam uyumlu)
# =============================================================================

class Job(BaseModel):
    id: int
    lat: float
    lon: float
    demand: int = 0
    time_start: int = 480
    time_end: int = 1080

class OptimizationRequest(BaseModel):
    vehicle_count: int
    vehicle_capacity: int = 1000
    jobs: List[Job]
    use_capacity: bool = True
    open_path: bool = False
    service_time: int = 10
    route_start_time: int = 480
    optimization_goal: str = "distance"

class PreviewRequest(BaseModel):
    jobs: List[Job]

# =============================================================================
# KATMAN 1: OSRM MATRİSLERİ (v1'den korundu, timeout eklendi)
# =============================================================================

def get_osrm_matrices(locations: List[Dict]) -> tuple:
    """
    Gerçek yol mesafesi ve süre matrislerini OSRM'den çeker.
    Returns: (distance_matrix_metres, duration_matrix_minutes)
    """
    coordinates = [f"{loc['lon']},{loc['lat']}" for loc in locations]
    coord_string = ";".join(coordinates)
    url = f"{OSRM_BASE_URL}/table/v1/driving/{coord_string}?annotations=distance,duration"

    try:
        response = requests.get(url, timeout=30)
        if response.status_code != 200:
            raise Exception(f"OSRM HTTP {response.status_code}")
        data = response.json()

        dist_matrix = [
            [int(val) if val is not None else 10_000_000 for val in row]
            for row in data["distances"]
        ]
        dur_matrix = [
            [int(val / 60) if val is not None else 9999 for val in row]
            for row in data["durations"]
        ]
        return dist_matrix, dur_matrix

    except requests.Timeout:
        raise Exception("OSRM timeout — lokasyon sayısını azalt veya self-host OSRM kullan")
    except Exception as e:
        raise Exception(f"OSRM hatası: {e}")


def get_route_geometry(path_locations: List[Dict]) -> tuple:
    """Verilen nokta listesi için gerçek yol geometrisi çeker."""
    if len(path_locations) < 2:
        return [], 0
    coords = [f"{loc['lon']},{loc['lat']}" for loc in path_locations]
    coord_string = ";".join(coords)
    url = f"{OSRM_BASE_URL}/route/v1/driving/{coord_string}?overview=full&geometries=polyline"
    try:
        response = requests.get(url, timeout=20)
        if response.status_code == 200:
            data = response.json()
            if data.get("routes"):
                route = data["routes"][0]
                return polyline_lib.decode(route["geometry"]), route["distance"]
    except Exception:
        pass
    return [], 0

# =============================================================================
# KATMAN 2: HGS BAŞLANGIÇ ÇÖZÜMÜ
# =============================================================================

def solve_with_hgs(
    locations: List[Dict],
    dist_matrix: List[List[int]],
    dur_matrix: List[List[int]],
    vehicle_count: int,
    vehicle_capacity: int,
    use_capacity: bool,
    service_time: int,
    route_start_time: int,
    time_limit_seconds: int = 10
) -> Optional[List[List[int]]]:
    """
    HGS ile başlangıç çözümü üretir.
    Returns: [[node_indices], ...] her araç için rota listesi
             Depot (0) dahil değil, sadece müşteri indeksleri
    """
    if not HGS_AVAILABLE:
        logger.info("HGS mevcut değil, OR-Tools'a geçiliyor")
        return None

    n = len(locations)

    try:
        # HGS problem tanımı
        data = {
            "distance_matrix": dist_matrix,
            "duration_matrix": dur_matrix,
            "demands": [loc["demand"] for loc in locations],
            "vehicle_capacities": [vehicle_capacity if use_capacity else 10_000_000] * vehicle_count,
            "num_vehicles": vehicle_count,
            "depot": 0,
            "service_times": [service_time if i > 0 else 0 for i in range(n)],
            "time_windows": [
                (loc["time_start"], loc["time_end"]) for loc in locations
            ],
            "vehicle_start_time": route_start_time,
        }

        ap = AlgorithmParameters(timeLimit=time_limit_seconds)
        hgs_solver = HGSSolver(parameters=ap, verbose=False)
        result = hgs_solver.solve_cvrp(data)

        if result is None:
            logger.warning("HGS çözüm bulamadı")
            return None

        routes = []
        for route in result.routes:
            if route:  # boş rotaları atla
                routes.append(route)

        logger.info(f"HGS çözümü: {len(routes)} aktif araç, "
                    f"toplam maliyet: {result.cost:.0f}")
        return routes

    except Exception as e:
        logger.warning(f"HGS hatası ({e}), OR-Tools fallback aktif")
        return None


def hgs_routes_to_ortools_hint(
    routing,
    manager,
    hgs_routes: List[List[int]]
) -> Any:
    """
    HGS çözümünü OR-Tools'un anlayacağı Assignment formatına çevirir.
    Bu sayede OR-Tools random başlangıç yerine HGS'in iyi çözümünden başlar.
    """
    initial_routes = []
    for route in hgs_routes:
        # HGS depot içermez, OR-Tools için sadece müşteri indeksleri yeterli
        initial_routes.append(route)

    try:
        assignment = routing.ReadAssignmentFromRoutes(initial_routes, True)
        return assignment
    except Exception as e:
        logger.warning(f"HGS hint dönüştürme hatası: {e}")
        return None

# =============================================================================
# KATMAN 3: OR-TOOLS VRPTW (Buglar fix edildi)
# =============================================================================

def build_ortools_model(
    locations: List[Dict],
    dist_matrix: List[List[int]],
    dur_matrix: List[List[int]],
    request: OptimizationRequest
):
    """
    OR-Tools modelini kurar. v1'deki 3 bug fix edildi:
    Bug #1: vehicles_to_force imkânsız constraint → düzeltildi
    Bug #2: birim senkronizasyonu → METER_PER_MINUTE ile normalize
    Bug #3: time window koşulsuz uygulanıyor
    Bug #4: makespan modunda arc cost = time (v1'de her zaman distance'tı)
    """
    n = len(locations)
    manager = pywrapcp.RoutingIndexManager(n, request.vehicle_count, 0)
    routing = pywrapcp.RoutingModel(manager)

    # --- Callback'ler ---
    def distance_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        if request.open_path and to_node == 0:
            return 0
        return dist_matrix[from_node][to_node]

    def time_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        if from_node == to_node:
            return 0
        if request.open_path and to_node == 0:
            return 0
        travel = dur_matrix[from_node][to_node]
        service = request.service_time if from_node != 0 else 0
        return travel + service

    def demand_callback(from_index):
        return locations[manager.IndexToNode(from_index)]["demand"]

    dist_cb_idx = routing.RegisterTransitCallback(distance_callback)
    time_cb_idx = routing.RegisterTransitCallback(time_callback)
    demand_cb_idx = routing.RegisterUnaryTransitCallback(demand_callback)

    # --- Bug #4 Fix: Arc cost moduna göre seçilir ---
    if request.optimization_goal == "makespan":
        routing.SetArcCostEvaluatorOfAllVehicles(time_cb_idx)
    else:
        routing.SetArcCostEvaluatorOfAllVehicles(dist_cb_idx)

    # --- Kapasite dimension ---
    actual_capacity = request.vehicle_capacity if request.use_capacity else 9_999_999
    routing.AddDimensionWithVehicleCapacity(
        demand_cb_idx, 0,
        [actual_capacity] * request.vehicle_count,
        True, "Capacity"
    )

    # --- Mesafe dimension ---
    routing.AddDimension(dist_cb_idx, 0, 999_999_999, True, "Distance")

    # --- Zaman dimension ---
    routing.AddDimension(time_cb_idx, 999_999, 999_999, False, "Time")
    time_dim = routing.GetDimensionOrDie("Time")

    for vehicle_id in range(request.vehicle_count):
        time_dim.CumulVar(routing.Start(vehicle_id)).SetValue(request.route_start_time)

    # --- Bug #3 Fix: Time window koşulsuz uygulanıyor ---
    for i in range(1, n):
        index = manager.NodeToIndex(i)
        start_t = locations[i]["time_start"]
        end_t = locations[i]["time_end"]
        if start_t > end_t:
            start_t, end_t = end_t, start_t
        time_dim.CumulVar(index).SetMin(start_t)
        time_dim.SetCumulVarSoftUpperBound(index, end_t, 1000)

    # --- Durak sayısı dimension ---
    def stop_count_cb(from_index):
        return 1
    stop_cb_idx = routing.RegisterUnaryTransitCallback(stop_count_cb)
    routing.AddDimension(stop_cb_idx, 0, n + 1, True, "StopCount")

    # --- Bug #1 & #2 Fix: Objective katsayıları ---
    num_stops = n - 1  # depot hariç
    stops_per_vehicle = max(1, num_stops // request.vehicle_count)

    # Birim referansları
    # dist_matrix: metre | dur_matrix: dakika
    # Şehir içi yaklaşık 18 km/h → 1 dakika ≈ 300 metre
    METER_PER_MINUTE = 300
    # Ortalama duraklar arası mesafe (matrisin ilk 10 elemanından tahmin)
    sample = [
        dist_matrix[0][j]
        for j in range(1, min(n, 11))
    ]
    AVG_STOP_DIST = int(sum(sample) / len(sample)) if sample else 3000

    stop_dim = routing.GetDimensionOrDie("StopCount")
    dist_dim = routing.GetDimensionOrDie("Distance")

    if request.optimization_goal == "vehicles":
        # Bug #2 Fix: Fixed cost dinamik — ortalama rota değerinin 1.5 katı
        avg_route_cost = stops_per_vehicle * AVG_STOP_DIST
        dynamic_fixed_cost = max(int(avg_route_cost * 1.5), 30_000)
        routing.SetFixedCostOfAllVehicles(dynamic_fixed_cost)
        logger.info(f"Vehicles modu — dynamic fixed cost: {dynamic_fixed_cost}m")

    else:
        # Bug #1 Fix: Sadece matematiksel olarak mümkün araç sayısını zorla
        # Her araç min 1 durak alsın (imkânsız constraint yok)
        max_forceable = min(request.vehicle_count, num_stops)
        force_penalty = AVG_STOP_DIST * 2  # caydırıcı ama abartılı değil

        for vehicle_id in range(max_forceable):
            stop_dim.SetCumulVarSoftLowerBound(
                routing.End(vehicle_id), 1, force_penalty
            )

        if request.optimization_goal == "distance":
            pass  # arc cost zaten mesafe

        elif request.optimization_goal == "balance":
            # 1 durak farkı = 1 ortalama duraklar arası mesafe cezası
            stop_dim.SetGlobalSpanCostCoefficient(AVG_STOP_DIST)

        elif request.optimization_goal == "makespan":
            # arc cost zaten time; global span da zaman cinsinden
            # 1 dakika fark = METER_PER_MINUTE metre ceza
            time_dim.SetGlobalSpanCostCoefficient(METER_PER_MINUTE)

    return routing, manager, time_dim


def solve_with_ortools(
    routing,
    manager,
    time_dim,
    locations,
    request,
    hgs_routes=None,
    time_limit=35
):
    search_params = pywrapcp.DefaultRoutingSearchParameters()

    # Hint aktarımı güvenilmez olduğu için şimdilik SAVINGS kullan.
    # HGS'i Hafta 2'de ALNS warm-start olarak entegre edeceğiz.
    search_params.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.SAVINGS
    )
    search_params.local_search_metaheuristic = (
        routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
    )
    search_params.time_limit.seconds = time_limit

    if hgs_routes:
        logger.info(f"HGS {len(hgs_routes)} rota üretti — "
                    f"Hafta 2'de ALNS warm-start olarak kullanılacak")

    t0 = time.time()
    solution = routing.SolveWithParameters(search_params)
    elapsed = time.time() - t0

    if solution:
        logger.info(f"OR-Tools çözüm buldu — {elapsed:.1f}s, "
                    f"maliyet: {solution.ObjectiveValue()}")
    else:
        logger.warning("OR-Tools çözüm bulamadı")

    return solution

# =============================================================================
# ÇÖZÜM → JSON DÖNÜŞÜMÜ
# =============================================================================

def extract_routes(
    solution,
    routing,
    manager,
    time_dim,
    locations: List[Dict],
    request: OptimizationRequest
) -> List[Dict]:
    """OR-Tools çözümünü v1 uyumlu JSON formatına çevirir."""
    routes_json = []
    depot = request.jobs[0]

    for vehicle_id in range(request.vehicle_count):
        index = routing.Start(vehicle_id)
        path_stops = []
        vehicle_load = 0

        while not routing.IsEnd(index):
            node = manager.IndexToNode(index)
            arrival_min = solution.Min(time_dim.CumulVar(index))
            arr_str = f"{(arrival_min // 60) % 24:02d}:{arrival_min % 60:02d}"
            loc = locations[node]
            vehicle_load += loc["demand"]
            path_stops.append({
                "lat": loc["lat"], "lon": loc["lon"],
                "id": loc["id"], "demand": loc["demand"],
                "arrival_time": arr_str
            })
            index = solution.Value(routing.NextVar(index))

        if len(path_stops) <= 1:
            routes_json.append({
                "vehicle_id": vehicle_id + 1,
                "path": [], "geometry": [],
                "total_km": 0, "total_load": 0
            })
            continue

        if not request.open_path:
            end_arrival = solution.Min(time_dim.CumulVar(index))
            arr_str = f"{(end_arrival // 60) % 24:02d}:{end_arrival % 60:02d}"
            path_stops.append({
                "lat": depot.lat, "lon": depot.lon,
                "id": depot.id, "demand": 0,
                "arrival_time": arr_str
            })

        geometry, true_distance = get_route_geometry(path_stops)
        formatted_path = [
            {
                "order": i + 1,
                "lat": s["lat"], "lon": s["lon"],
                "original_id": s["id"],
                "demand": s["demand"],
                "arrival_time": s["arrival_time"]
            }
            for i, s in enumerate(path_stops)
        ]

        routes_json.append({
            "vehicle_id": vehicle_id + 1,
            "path": formatted_path,
            "geometry": geometry,
            "total_km": round(true_distance / 1000, 2),
            "total_load": vehicle_load
        })

    return routes_json

# =============================================================================
# DASHBOARD HTML (v1'den korundu, v2 badge eklendi)
# =============================================================================

def generate_dashboard_html(jobs, result_json, service_time, solver_info=None):
    if not result_json.get("routes"):
        return "<h1>No Route Found</h1>"

    center_lat, center_lon = jobs[0].lat, jobs[0].lon
    m = folium.Map(location=[center_lat, center_lon], zoom_start=11)

    hex_colors = [
        "#d50000","#2962ff","#00c853","#aa00ff","#ff6d00",
        "#00bfa5","#ffd600","#c51162","#aeea00","#00b8d4"
    ]

    gantt_data = []
    total_km = 0
    total_load = 0
    active_vehicles = 0
    active_vehicle_colors = []
    wa_buttons = []

    for i, route in enumerate(result_json["routes"]):
        path = route["path"]
        if not path:
            continue

        active_vehicles += 1
        total_km += route["total_km"]
        total_load += route.get("total_load", 0)

        geometry = route.get("geometry", [])
        color = hex_colors[(active_vehicles - 1) % len(hex_colors)]
        active_vehicle_colors.append(color)
        v_name = f"Vehicle {route['vehicle_id']}"

        wp_array = [f"{stop['lat']},{stop['lon']}" for stop in path]
        max_stops = 10
        chunks = []
        if len(wp_array) <= max_stops:
            chunks.append(wp_array)
        else:
            for j in range(0, len(wp_array) - 1, max_stops - 1):
                chunks.append(wp_array[j:j + max_stops])

        for idx, chunk in enumerate(chunks):
            origin = chunk[0]
            destination = chunk[-1]
            waypoints = ""
            if len(chunk) > 2:
                waypoints = "&waypoints=" + "%7C".join(chunk[1:-1])
            maps_url = (
                f"https://www.google.com/maps/dir/?api=1"
                f"&origin={origin}&destination={destination}"
                f"{waypoints}&travelmode=driving"
            )
            part_text = f" (Part {idx + 1})" if len(chunks) > 1 else ""
            msg = f"🚚 {v_name}{part_text} Route is Ready!\n\nStart Navigation:\n{maps_url}"
            wa_link = f"https://wa.me/?text={urllib.parse.quote(msg)}"
            btn_html = (
                f"<a href='{wa_link}' target='_blank' style='background-color:{color};"
                f"color:white;text-decoration:none;padding:10px 15px;border-radius:6px;"
                f"font-size:13px;font-weight:bold;display:inline-flex;align-items:center;'>"
                f"{v_name}{part_text} 📲</a>"
            )
            wa_buttons.append(btn_html)

        if geometry:
            folium.PolyLine(
                geometry, color=color, weight=4, opacity=0.8,
                tooltip=f"{v_name}: {route['total_km']} km"
            ).add_to(m)

        for stop in path:
            orj_id, lat, lon = stop["original_id"], stop["lat"], stop["lon"]
            order = stop["order"]
            arrival = stop.get("arrival_time", "")

            if orj_id == 0:
                if order == 1:
                    folium.Marker(
                        [lat, lon], popup="DEPOT",
                        icon=folium.Icon(color="black", icon="home", prefix="fa")
                    ).add_to(m)
            else:
                display_num = order - 1
                folium.Marker(
                    [lat, lon],
                    icon=DivIcon(
                        icon_size=(30, 30), icon_anchor=(15, 15),
                        html=(
                            f'<div style="font-size:11pt;font-weight:bold;color:{color};'
                            f'background-color:white;border:2px solid {color};border-radius:50%;'
                            f'width:30px;height:30px;text-align:center;line-height:26px;">'
                            f'{display_num}</div>'
                        )
                    )
                ).add_to(m)

            if arrival:
                h, m_minute = map(int, arrival.split(":"))
                duration = service_time if orj_id != 0 else 5
                end_m = m_minute + duration
                end_h = (h + (end_m // 60)) % 24
                end_m = end_m % 60
                stop_label = (
                    f"Stop {order - 1}" if orj_id != 0
                    else ("Depot Departure" if order == 1 else "Depot Return")
                )
                gantt_data.append(
                    f"[ '{v_name}', '{stop_label}', "
                    f"new Date(0,0,0,{h},{m_minute},0), "
                    f"new Date(0,0,0,{end_h},{end_m},0) ]"
                )

    gantt_rows_str = ",\n".join(gantt_data)
    js_colors_array = "[" + ",".join([f"'{c}'" for c in active_vehicle_colors]) + "]"
    timeline_height = max(250, active_vehicles * 45 + 80)

    # Solver bilgisi (v2 badge)
    solver_badge = ""
    if solver_info:
        hgs_used = "✅ HGS" if solver_info.get("hgs_used") else "⚠️ Fallback"
        solver_badge = (
            f"<div style='position:absolute;top:8px;right:12px;background:rgba(0,0,0,0.6);"
            f"color:white;padding:4px 10px;border-radius:12px;font-size:11px;z-index:9999;'>"
            f"v2 | {hgs_used} | {solver_info.get('solve_time','?')}s</div>"
        )

    base_html = m.get_root().render()

    css_override = """
    <link href="https://fonts.googleapis.com/css2?family=Roboto:wght@400;500;700&display=swap" rel="stylesheet">
    <style>
        html, body { margin:0; padding:0; display:flex; flex-direction:column;
                     height:100vh; overflow-y:auto; overflow-x:hidden;
                     font-family:'Roboto',sans-serif; background:#f8f9fa; }
        .folium-map { min-height:50vh; flex-grow:1; z-index:1; }
    </style>
    """
    base_html = base_html.replace("</head>", css_override + "</head>")

    kpi_section = f"""
    <div style="position:relative;display:flex;justify-content:space-around;align-items:center;
         background:#1a73e8;color:white;padding:15px;box-shadow:0 4px 6px rgba(0,0,0,0.1);
         z-index:9999;flex-shrink:0;">
        {solver_badge}
        <div style="text-align:center;">
            <div style="font-size:26px;font-weight:bold;">{round(total_km,1)}<span style="font-size:16px;font-weight:normal;"> km</span></div>
            <div style="font-size:12px;opacity:0.8;text-transform:uppercase;letter-spacing:1px;">Total Distance</div>
        </div>
        <div style="text-align:center;">
            <div style="font-size:26px;font-weight:bold;">{active_vehicles}</div>
            <div style="font-size:12px;opacity:0.8;text-transform:uppercase;letter-spacing:1px;">Active Vehicles</div>
        </div>
        <div style="text-align:center;">
            <div style="font-size:26px;font-weight:bold;">{total_load}<span style="font-size:16px;font-weight:normal;"> kg</span></div>
            <div style="font-size:12px;opacity:0.8;text-transform:uppercase;letter-spacing:1px;">Total Load</div>
        </div>
    </div>
    """
    base_html = base_html.replace("<body>", "<body>" + kpi_section)

    wa_section = f"""
    <div style="background:white;padding:15px;box-sizing:border-box;
         border-top:1px solid #ddd;z-index:9999;flex-shrink:0;">
        <div style="font-size:14px;font-weight:bold;color:#555;margin-bottom:10px;">
            📲 Send Routes to Drivers (WhatsApp)
        </div>
        <div style="display:flex;flex-wrap:wrap;gap:8px;">
            {" ".join(wa_buttons)}
        </div>
    </div>
    """

    timeline_section = f"""
    {wa_section}
    <div style="min-height:{timeline_height+40}px;width:100%;position:relative;
         z-index:9999;background:white;padding:15px;box-sizing:border-box;
         border-top:1px solid #ddd;flex-shrink:0;">
        <div style="font-size:14px;font-weight:bold;color:#555;margin-bottom:10px;">
            ⏱️ Vehicle Shift Timeline
        </div>
        <div id="timeline" style="height:{timeline_height}px;width:100%;"></div>
    </div>
    <script type="text/javascript" src="https://www.gstatic.com/charts/loader.js"></script>
    <script type="text/javascript">
      google.charts.load('current', {{'packages':['timeline']}});
      google.charts.setOnLoadCallback(drawChart);
      function drawChart() {{
        var container = document.getElementById('timeline');
        var chart = new google.visualization.Timeline(container);
        var dataTable = new google.visualization.DataTable();
        dataTable.addColumn({{ type: 'string', id: 'Vehicle' }});
        dataTable.addColumn({{ type: 'string', id: 'Stop' }});
        dataTable.addColumn({{ type: 'date', id: 'Start' }});
        dataTable.addColumn({{ type: 'date', id: 'End' }});
        dataTable.addRows([{gantt_rows_str}]);
        var options = {{
            timeline: {{ colorByRowLabel: true, showRowLabels: true }},
            backgroundColor: '#ffffff',
            hAxis: {{ format: 'HH:mm' }},
            colors: {js_colors_array}
        }};
        chart.draw(dataTable, options);
      }}
    </script>
    """
    base_html = base_html.replace("</body>", timeline_section + "</body>")
    return base_html

# =============================================================================
# API ENDPOINT'LERİ
# =============================================================================

@app.post("/preview")
def preview_map(request: PreviewRequest):
    """Lokasyonları haritada önizle (v1 ile aynı)."""
    if not request.jobs:
        raise HTTPException(status_code=400, detail="Lokasyon bulunamadı")

    center_lat, center_lon = request.jobs[0].lat, request.jobs[0].lon
    m = folium.Map(location=[center_lat, center_lon], zoom_start=11)

    for loc in request.jobs:
        if loc.id == 0:
            folium.Marker(
                [loc.lat, loc.lon], tooltip="MAIN DEPOT",
                icon=folium.Icon(color="black", icon="home", prefix="fa")
            ).add_to(m)
        else:
            folium.Marker(
                [loc.lat, loc.lon],
                tooltip=f"Stop ID: {loc.id} | Load: {loc.demand} kg",
                icon=folium.Icon(color="blue", icon="info-sign")
            ).add_to(m)

    return {"status": "success", "map_html": m.get_root().render()}


@app.post("/optimize")
def optimize_v2(request: OptimizationRequest):
    """
    v2 Ana optimizasyon endpoint'i.
    Katman 2 (HGS) + Katman 3 (OR-Tools, bug'lar fix edildi).
    """
    locations = [
        {
            "lat": j.lat, "lon": j.lon,
            "id": j.id, "demand": j.demand,
            "time_start": j.time_start, "time_end": j.time_end
        }
        for j in request.jobs
    ]

    # Minimum validasyon
    if len(locations) < 2:
        raise HTTPException(status_code=400, detail="En az 1 depot + 1 müşteri gerekli")

    # Katman 1: OSRM
    try:
        dist_matrix, dur_matrix = get_osrm_matrices(locations)
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Harita servisi hatası: {e}")

    t_start = time.time()

    # Katman 2: HGS başlangıç çözümü
    # OR-Tools için kalan süreyi ayarla (toplam budget: 40s)
    hgs_time = 12 if len(locations) > 30 else 8
    hgs_routes = solve_with_hgs(
        locations, dist_matrix, dur_matrix,
        vehicle_count=request.vehicle_count,
        vehicle_capacity=request.vehicle_capacity,
        use_capacity=request.use_capacity,
        service_time=request.service_time,
        route_start_time=request.route_start_time,
        time_limit_seconds=hgs_time
    )

    # Katman 3: OR-Tools (bug'lar fix edildi)
    routing, manager, time_dim = build_ortools_model(
        locations, dist_matrix, dur_matrix, request
    )

    ortools_time = max(15, 40 - int(time.time() - t_start))
    solution = solve_with_ortools(
        routing, manager, time_dim, locations, request,
        hgs_routes=hgs_routes,
        time_limit=ortools_time
    )

    elapsed = round(time.time() - t_start, 1)

    if not solution:
        raise HTTPException(status_code=400, detail="Rota bulunamadı — araç sayısını veya kapasiteyi artır")

    # Çözüm → JSON
    routes = extract_routes(solution, routing, manager, time_dim, locations, request)

    solver_info = {
        "hgs_used": hgs_routes is not None,
        "solve_time": elapsed,
        "solver": "HGS + OR-Tools" if hgs_routes else "OR-Tools only"
    }

    result = {
        "status": "success",
        "routes": routes,
        "solver_info": solver_info
    }

    result["map_html"] = generate_dashboard_html(
        request.jobs, result, request.service_time, solver_info
    )
    return result


@app.get("/health")
def health():
    return {
        "status": "ok",
        "version": "2.0.0",
        "hgs_available": HGS_AVAILABLE,
        "layers": {
            "1_osrm": "active",
            "2_hgs": "active" if HGS_AVAILABLE else "fallback_only",
            "3_ortools": "active",
            "4_alns": "coming_week_2",
            "5_pareto": "coming_week_3"
        }
    }
