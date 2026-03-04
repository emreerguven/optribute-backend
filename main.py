from fastapi import FastAPI, HTTPException
from pydantic import BaseModel
from typing import List, Tuple
from folium.features import DivIcon
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
import requests
import folium
import polyline
import math

app = FastAPI()

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

class PreviewRequest(BaseModel):
    jobs: List[Job]

OSRM_BASE_URL = "http://router.project-osrm.org"
SERVICE_TIME = 10

# ─────────────────────────────────────────────
# OSRM
# ─────────────────────────────────────────────
def get_osrm_matrices(locations):
    coordinates = [f"{loc['lon']},{loc['lat']}" for loc in locations]
    coord_string = ";".join(coordinates)
    url = f"{OSRM_BASE_URL}/table/v1/driving/{coord_string}?annotations=distance,duration"
    response = requests.get(url, timeout=15)
    if response.status_code != 200:
        raise Exception("OSRM Hatası")
    data = response.json()
    dist_matrix = [[int(v) if v is not None else 10_000_000 for v in row] for row in data["distances"]]
    dur_matrix  = [[int(v / 60) if v is not None else 9_999 for v in row] for row in data["durations"]]
    return dist_matrix, dur_matrix

def get_route_geometry(path_locations):
    if len(path_locations) < 2:
        return [], 0
    coords = [f"{loc['lon']},{loc['lat']}" for loc in path_locations]
    url = f"{OSRM_BASE_URL}/route/v1/driving/{';'.join(coords)}?overview=full&geometries=polyline"
    try:
        response = requests.get(url, timeout=15)
        if response.status_code == 200:
            data = response.json()
            if data.get("routes"):
                route = data["routes"][0]
                return polyline.decode(route["geometry"]), route["distance"]
    except Exception:
        pass
    return [], 0

# ─────────────────────────────────────────────
# ANTİ-SPAGETTİ: 2-OPT CROSSING REMOVAL
# Kağıt üzerinde iki kenar kesişiyorsa swap yap
# ─────────────────────────────────────────────
def _segments_intersect(p1, p2, p3, p4) -> bool:
    """İki line segment kesişiyor mu? (lat/lon koordinatlarında)"""
    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    d1 = cross(p3, p4, p1)
    d2 = cross(p3, p4, p2)
    d3 = cross(p1, p2, p3)
    d4 = cross(p1, p2, p4)

    if ((d1 > 0 < d2) or (d1 < 0 > d2)) and \
       ((d3 > 0 < d4) or (d3 < 0 > d4)):
        return True
    return False

def euclidean(a, b) -> float:
    return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)

def two_opt_single_route(route: List[dict], max_passes: int = 3) -> List[dict]:
    """
    Tek rota içi 2-opt.
    max_passes: 50+ durakta sonsuz döngüyü önler, 3 geçiş yeterli.
    """
    coords = [(s["lat"], s["lon"]) for s in route]
    n = len(coords)

    for _ in range(max_passes):
        improved = False
        for i in range(1, n - 2):
            for j in range(i + 1, n - 1):
                if _segments_intersect(
                    coords[i-1], coords[i], coords[j], coords[j+1]
                ):
                    coords[i:j+1] = coords[i:j+1][::-1]
                    route[i:j+1]  = route[i:j+1][::-1]
                    improved = True
        if not improved:
            break
    return route

def or_opt_single_route(route: List[dict], max_passes: int = 2) -> List[dict]:
    """
    Or-opt: tek durakları daha ucuz yere taşı.
    max_passes: 2 geçiş 50+ durakta makul sürede biter.
    """
    coords = [(s["lat"], s["lon"]) for s in route]
    n = len(coords)

    for _ in range(max_passes):
        improved = False
        for i in range(1, n - 1):
            ci     = coords[i]
            prev_i = coords[i - 1]
            next_i = coords[i + 1]
            current_cost = euclidean(prev_i, ci) + euclidean(ci, next_i)
            gap_cost     = euclidean(prev_i, next_i)
            savings      = current_cost - gap_cost  # bu durak kaldırılınca kazanç

            best_gain = 0
            best_j    = -1

            for j in range(1, n - 1):
                if abs(j - i) <= 1:
                    continue
                cj      = coords[j]
                cj_next = coords[min(j + 1, n - 1)]
                insert_cost = (
                    euclidean(cj, ci) + euclidean(ci, cj_next)
                    - euclidean(cj, cj_next)
                )
                gain = savings - insert_cost
                if gain > best_gain:
                    best_gain = gain
                    best_j    = j

            if best_j != -1:
                node = route.pop(i)
                coords.pop(i)
                insert_pos = best_j if best_j < i else best_j
                route.insert(insert_pos, node)
                coords.insert(insert_pos, (node["lat"], node["lon"]))
                improved = True
                break   # Bu geçişte bir taşıma yaptık, başa dön

        if not improved:
            break
    return route

def inter_route_two_opt(routes_stops: List[List[dict]]) -> List[List[dict]]:
    """
    Rotalar ARASI kesişim tespiti.
    max_iter=5 ile sınırlandırıldı — 50+ durakta kabul edilebilir süre.
    """
    n_routes = len(routes_stops)
    max_iter = 5
    iteration = 0
    improved  = True

    while improved and iteration < max_iter:
        improved  = False
        iteration += 1

        for r1 in range(n_routes):
            for r2 in range(r1 + 1, n_routes):
                s1 = routes_stops[r1]
                s2 = routes_stops[r2]
                if len(s1) < 2 or len(s2) < 2:
                    continue

                found = False
                for i in range(len(s1) - 1):
                    if found:
                        break
                    p1 = (s1[i]["lat"],   s1[i]["lon"])
                    p2 = (s1[i+1]["lat"], s1[i+1]["lon"])

                    for j in range(len(s2) - 1):
                        p3 = (s2[j]["lat"],   s2[j]["lon"])
                        p4 = (s2[j+1]["lat"], s2[j+1]["lon"])

                        if _segments_intersect(p1, p2, p3, p4):
                            tail1 = s1[i+1:]
                            tail2 = s2[j+1:]
                            routes_stops[r1] = s1[:i+1] + tail2
                            routes_stops[r2] = s2[:j+1] + tail1
                            improved = True
                            found    = True
                            break

    return routes_stops

# ─────────────────────────────────────────────
# HARİTA
# ─────────────────────────────────────────────
def generate_map_html(jobs, result_json):
    if not result_json.get("routes"):
        return "<h1>Rota yok</h1>"
    center_lat, center_lon = jobs[0].lat, jobs[0].lon
    m = folium.Map(location=[center_lat, center_lon], zoom_start=11)
    colors = ["red","blue","green","purple","orange","darkred",
              "cadetblue","darkblue","darkgreen","pink","lightred","beige"]

    for i, route in enumerate(result_json["routes"]):
        path      = route["path"]
        geometry  = route.get("geometry", [])
        color     = colors[i % len(colors)]
        total_load = route.get("total_load", 0)
        if not path:
            continue

        if geometry:
            folium.PolyLine(
                geometry, color=color, weight=4, opacity=0.8,
                tooltip=f"Araç {route['vehicle_id']}: {route['total_km']} km | Yük: {total_load} kg"
            ).add_to(m)

        for stop in path:
            orj_id, lat, lon = stop["original_id"], stop["lat"], stop["lon"]
            order  = stop["order"]
            demand = stop.get("demand", 0)
            arrival = stop.get("arrival_time", "")

            if orj_id == 0:
                if order == 1:
                    folium.Marker(
                        [lat, lon], popup="DEPO (Çıkış)",
                        icon=folium.Icon(color="black", icon="home", prefix="fa")
                    ).add_to(m)
            else:
                display_num = order - 1
                folium.Marker(
                    [lat, lon],
                    icon=DivIcon(
                        icon_size=(30, 30), icon_anchor=(15, 15),
                        html=f'<div style="font-size:11pt;font-weight:bold;color:{color};'
                             f'background:white;border:2px solid {color};border-radius:50%;'
                             f'width:30px;height:30px;text-align:center;line-height:26px;">'
                             f'{display_num}</div>'
                    )
                ).add_to(m)
                folium.Marker(
                    [lat, lon],
                    tooltip=f"Durak {display_num}: {demand} Kg | Saat: {arrival}",
                    opacity=0
                ).add_to(m)
    return m.get_root().render()

# ─────────────────────────────────────────────
# ENDPOINTS
# ─────────────────────────────────────────────
@app.post("/preview")
def preview_map(request: PreviewRequest):
    locations = request.jobs
    if not locations:
        raise HTTPException(status_code=400, detail="Konum bulunamadı")

    center_lat = locations[0].lat
    center_lon = locations[0].lon
    m = folium.Map(location=[center_lat, center_lon], zoom_start=11)

    for loc in locations:
        if loc.id == 0:
            folium.Marker(
                [loc.lat, loc.lon], tooltip="MERKEZ DEPO",
                icon=folium.Icon(color="black", icon="home", prefix="fa")
            ).add_to(m)
        else:
            folium.Marker(
                [loc.lat, loc.lon],
                tooltip=f"Durak ID: {loc.id} | Yük: {loc.demand} kg",
                icon=folium.Icon(color="blue", icon="info-sign")
            ).add_to(m)

    return {"status": "success", "map_html": m.get_root().render()}


@app.post("/optimize")
def optimize(request: OptimizationRequest):
    locations = [
        {
            "lat": j.lat, "lon": j.lon, "id": j.id,
            "demand": j.demand,
            "time_start": j.time_start, "time_end": j.time_end
        }
        for j in request.jobs
    ]

    try:
        distance_matrix, duration_matrix = get_osrm_matrices(locations)
    except Exception:
        raise HTTPException(status_code=500, detail="Harita servisi hatası")

    n          = len(locations)
    k          = request.vehicle_count

    # ── OR-Tools kurulumu ──────────────────────────────────
    manager = pywrapcp.RoutingIndexManager(n, k, 0)
    routing = pywrapcp.RoutingModel(manager)

    # Mesafe callback
    def distance_callback(from_index, to_index):
        fn = manager.IndexToNode(from_index)
        tn = manager.IndexToNode(to_index)
        if request.open_path and tn == 0:
            return 0
        return distance_matrix[fn][tn]

    transit_cb = routing.RegisterTransitCallback(distance_callback)
    routing.SetArcCostEvaluatorOfAllVehicles(transit_cb)

    # Kapasite callback
    def demand_callback(from_index):
        return locations[manager.IndexToNode(from_index)]["demand"]

    demand_cb = routing.RegisterUnaryTransitCallback(demand_callback)
    actual_capacity = request.vehicle_capacity if request.use_capacity else 10_000_000
    routing.AddDimensionWithVehicleCapacity(
        demand_cb, 0, [actual_capacity] * k, True, "Capacity"
    )

    # Zaman callback
    def time_callback(from_index, to_index):
        fn = manager.IndexToNode(from_index)
        tn = manager.IndexToNode(to_index)
        if fn == tn:
            return 0
        if request.open_path and tn == 0:
            return 0
        return duration_matrix[fn][tn] + SERVICE_TIME

    time_cb = routing.RegisterTransitCallback(time_callback)
    routing.AddDimension(time_cb, 9_999, 9_999, False, "Time")
    time_dim = routing.GetDimensionOrDie("Time")

    for vid in range(k):
        time_dim.CumulVar(routing.Start(vid)).SetValue(480)

    for i in range(1, n):
        idx = manager.NodeToIndex(i)
        ts, te = locations[i]["time_start"], locations[i]["time_end"]
        if ts > te:
            ts, te = te, ts
        # Soft time window — ceza katsayısı 200
        time_dim.CumulVar(idx).SetMin(ts)
        time_dim.SetCumulVarSoftUpperBound(idx, te, 200)

    # Mesafe dimension — makespan dengelemek için
    routing.AddDimension(transit_cb, 0, 999_999_999, True, "Distance")
    dist_dim = routing.GetDimensionOrDie("Distance")

    # ★ MAKESPAN DENGELEYİCİ
    # 0 → tamamen görmezden gel (eski), 150 → dengeli
    dist_dim.SetGlobalSpanCostCoefficient(150)

    # ── Arama parametreleri ────────────────────────────────
    params = pywrapcp.DefaultRoutingSearchParameters()

    # PATH_CHEAPEST_ARC coğrafi lokalite sağlar → daha az crossing
    params.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    )
    # GLS en iyi local search — değiştirme
    params.local_search_metaheuristic = (
        routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
    )
    params.time_limit.seconds = 20
    params.solution_limit      = 100

    solution = routing.SolveWithParameters(params)

    if not solution:
        raise HTTPException(status_code=400, detail="Rota bulunamadı!")

    # ── Ham rotaları çıkar ────────────────────────────────
    depot = request.jobs[0]
    raw_routes: List[List[dict]] = []

    for vid in range(k):
        idx    = routing.Start(vid)
        stops  = []
        v_load = 0

        while not routing.IsEnd(idx):
            arr_min = solution.Min(time_dim.CumulVar(idx))
            arr_str = f"{(arr_min // 60) % 24:02d}:{arr_min % 60:02d}"
            loc     = locations[manager.IndexToNode(idx)]
            v_load += loc["demand"]
            stops.append({
                "lat": loc["lat"], "lon": loc["lon"],
                "id":  loc["id"],  "demand": loc["demand"],
                "arrival_time": arr_str
            })
            idx = solution.Value(routing.NextVar(idx))

        if len(stops) <= 1:
            raw_routes.append([])
            continue

        if not request.open_path:
            arr_min = solution.Min(time_dim.CumulVar(idx))
            arr_str = f"{(arr_min // 60) % 24:02d}:{arr_min % 60:02d}"
            stops.append({
                "lat": depot.lat, "lon": depot.lon,
                "id": depot.id, "demand": 0,
                "arrival_time": arr_str
            })

        raw_routes.append(stops)

    # ── POST-PROCESSING: Anti-Spaghetti ───────────────────

    # 1. Her rota kendi içi 2-opt (iç kesişimler)
    non_empty = [r for r in raw_routes if r]
    processed = [two_opt_single_route(r) for r in non_empty]

    # 2. Or-opt: durakları daha iyi yere taşı
    processed = [or_opt_single_route(r) for r in processed]

    # 3. Rotalar arası crossing removal
    processed = inter_route_two_opt(processed)

    # ── Sonucu formatla ──────────────────────────────────
    routes_json = []
    proc_idx    = 0

    for vid in range(k):
        if not raw_routes[vid]:
            routes_json.append({
                "vehicle_id": vid + 1,
                "path": [], "geometry": [], "total_km": 0, "total_load": 0
            })
            continue

        stops     = processed[proc_idx]
        proc_idx += 1
        v_load    = sum(s["demand"] for s in stops)

        geometry, true_distance = get_route_geometry(stops)

        formatted = [
            {
                "order":       i + 1,
                "lat":         s["lat"],
                "lon":         s["lon"],
                "original_id": s["id"],
                "demand":      s["demand"],
                "arrival_time": s["arrival_time"]
            }
            for i, s in enumerate(stops)
        ]

        routes_json.append({
            "vehicle_id": vid + 1,
            "path":       formatted,
            "geometry":   geometry,
            "total_km":   round(true_distance / 1000, 2),
            "total_load": v_load
        })

    result = {"status": "success", "routes": routes_json}
    result["map_html"] = generate_map_html(request.jobs, result)
    return result
