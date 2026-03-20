from fastapi import FastAPI, HTTPException
from pydantic import BaseModel
from typing import List
from folium.features import DivIcon
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
import urllib.parse
import requests
import folium
import polyline

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
    service_time: int = 10        
    route_start_time: int = 480   
    optimization_goal: str = "distance" 

class PreviewRequest(BaseModel):
    jobs: List[Job]

OSRM_BASE_URL = "http://router.project-osrm.org"

def get_osrm_matrices(locations):
    coordinates = [f"{loc['lon']},{loc['lat']}" for loc in locations]
    coord_string = ";".join(coordinates)
    url = f"{OSRM_BASE_URL}/table/v1/driving/{coord_string}?annotations=distance,duration"
    try:
        response = requests.get(url)
        if response.status_code != 200: raise Exception("OSRM Error")
        data = response.json()
        dist_matrix = [[int(val) if val is not None else 1000000 for val in row] for row in data["distances"]]
        dur_matrix = [[int(val/60) if val is not None else 999 for val in row] for row in data["durations"]]
        return dist_matrix, dur_matrix
    except Exception as e:
        raise e

def get_route_geometry(path_locations):
    if len(path_locations) < 2: return [], 0
    coords = [f"{loc['lon']},{loc['lat']}" for loc in path_locations]
    coord_string = ";".join(coords)
    url = f"{OSRM_BASE_URL}/route/v1/driving/{coord_string}?overview=full&geometries=polyline"
    try:
        response = requests.get(url)
        if response.status_code == 200:
            data = response.json()
            if "routes" in data and len(data["routes"]) > 0:
                route = data["routes"][0]
                return polyline.decode(route["geometry"]), route["distance"]
    except: pass
    return [], 0

def generate_dashboard_html(jobs, result_json, service_time):
    if not result_json.get("routes"): return "<h1>No Route Found</h1>"
    center_lat, center_lon = jobs[0].lat, jobs[0].lon
    m = folium.Map(location=[center_lat, center_lon], zoom_start=11)
    
    hex_colors = ["#d50000", "#2962ff", "#00c853", "#aa00ff", "#ff6d00", "#00bfa5", "#ffd600", "#c51162", "#aeea00", "#00b8d4"]

    gantt_data = []
    total_km = 0
    total_load = 0
    active_vehicles = 0
    active_vehicle_colors = []
    wa_buttons = []

    for i, route in enumerate(result_json["routes"]):
        path = route["path"]
        if not path: continue

        active_vehicles += 1
        total_km += route["total_km"]
        current_route_load = route.get("total_load", 0)
        total_load += current_route_load

        geometry = route.get("geometry", [])
        
        color = hex_colors[(active_vehicles - 1) % len(hex_colors)]
        active_vehicle_colors.append(color)
        
        v_name = f"Vehicle {route['vehicle_id']}"

        wp_array = [f"{stop['lat']},{stop['lon']}" for stop in path]
        if len(wp_array) > 0:
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
                
                maps_url = f"https://www.google.com/maps/dir/?api=1&origin={origin}&destination={destination}{waypoints}&travelmode=driving"
                
                part_text = f" (Part {idx + 1})" if len(chunks) > 1 else ""
                msg = f"🚚 {v_name}{part_text} Route is Ready!\n\nStart Navigation:\n{maps_url}"
                wa_link = f"https://wa.me/?text={urllib.parse.quote(msg)}"
                
                btn_html = f"<a href='{wa_link}' target='_blank' style='background-color: {color}; color: white; text-decoration: none; padding: 10px 15px; border-radius: 6px; font-size: 13px; font-weight: bold; box-shadow: 0 2px 4px rgba(0,0,0,0.15); display: inline-flex; align-items: center; justify-content: center;'>{v_name}{part_text} 📲</a>"
                wa_buttons.append(btn_html)

        if geometry:
            folium.PolyLine(geometry, color=color, weight=4, opacity=0.8, tooltip=f"{v_name}: {route['total_km']} km").add_to(m)

        for stop in path:
            orj_id, lat, lon = stop["original_id"], stop["lat"], stop["lon"]
            order, demand, arrival = stop["order"], stop.get("demand", 0), stop.get("arrival_time", "")

            if orj_id == 0:
                if order == 1:
                    folium.Marker([lat, lon], popup="DEPOT", icon=folium.Icon(color="black", icon="home", prefix="fa")).add_to(m)
            else:
                display_num = order - 1
                folium.Marker(
                    [lat, lon],
                    icon=DivIcon(
                        icon_size=(30, 30), icon_anchor=(15, 15),
                        html=f'<div style="font-size: 11pt; font-weight: bold; color: {color}; background-color: white; border: 2px solid {color}; border-radius: 50%; width: 30px; height: 30px; text-align: center; line-height: 26px;">{display_num}</div>'
                    )
                ).add_to(m)

            if arrival:
                h, m_minute = map(int, arrival.split(":"))
                duration = service_time if orj_id != 0 else 5 
                end_m = m_minute + duration
                end_h = (h + (end_m // 60)) % 24
                end_m = end_m % 60

                stop_label = f"Stop {order - 1}" if orj_id != 0 else ("Depot Departure" if order == 1 else "Depot Return")
                gantt_data.append(f"[ '{v_name}', '{stop_label}', new Date(0,0,0, {h},{m_minute},0), new Date(0,0,0, {end_h},{end_m},0) ]")

    gantt_rows_str = ",\n".join(gantt_data)
    js_colors_array = "[" + ",".join([f"'{c}'" for c in active_vehicle_colors]) + "]"
    
    timeline_height = max(250, active_vehicles * 45 + 80)
    
    base_html = m.get_root().render()

    css_override = """
    <link href="https://fonts.googleapis.com/css2?family=Roboto:wght@400;500;700&display=swap" rel="stylesheet">
    <style>
        html, body { margin: 0; padding: 0; display: flex; flex-direction: column; height: 100vh; overflow-y: auto; overflow-x: hidden; font-family: 'Roboto', sans-serif; background-color: #f8f9fa;}
        .folium-map { min-height: 50vh; flex-grow: 1; z-index: 1;}
    </style>
    """
    base_html = base_html.replace("</head>", css_override + "</head>")

    kpi_section = f"""
    <div style="display:flex; justify-content:space-around; align-items:center; background:#1a73e8; color:white; padding:15px; box-shadow: 0 4px 6px rgba(0,0,0,0.1); z-index:9999; position:relative; flex-shrink: 0;">
        <div style="text-align:center;"><div style="font-size:26px; font-weight:bold;">{round(total_km,1)} <span style="font-size:16px;font-weight:normal;">km</span></div><div style="font-size:12px; opacity:0.8; text-transform:uppercase; letter-spacing:1px;">Total Distance</div></div>
        <div style="text-align:center;"><div style="font-size:26px; font-weight:bold;">{active_vehicles}</div><div style="font-size:12px; opacity:0.8; text-transform:uppercase; letter-spacing:1px;">Active Vehicles</div></div>
        <div style="text-align:center;"><div style="font-size:26px; font-weight:bold;">{total_load} <span style="font-size:16px;font-weight:normal;">kg</span></div><div style="font-size:12px; opacity:0.8; text-transform:uppercase; letter-spacing:1px;">Total Load</div></div>
    </div>
    """
    base_html = base_html.replace("<body>", "<body>" + kpi_section)

    wa_buttons_html = f"""
    <div style="background: white; padding: 15px; box-sizing: border-box; border-top: 1px solid #ddd; z-index:9999; position:relative; flex-shrink: 0;">
        <div style="font-size:14px; font-weight:bold; color:#555; margin-bottom:10px;">📲 Send Routes to Drivers (WhatsApp)</div>
        <div style="display: flex; flex-wrap: wrap; gap: 8px;">
            {" ".join(wa_buttons)}
        </div>
    </div>
    """

    timeline_section = f"""
    {wa_buttons_html}
    <div id="timeline_container" style="min-height: {timeline_height + 40}px; width: 100%; position: relative; z-index:9999; background: white; padding: 15px; box-sizing: border-box; border-top: 1px solid #ddd; box-shadow: 0 -2px 10px rgba(0,0,0,0.05); flex-shrink: 0;">
        <div style="font-size:14px; font-weight:bold; color:#555; margin-bottom:10px;">⏱️ Vehicle Shift Timeline</div>
        <div id="timeline" style="height: {timeline_height}px; width: 100%;"></div>
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
        dataTable.addRows([
          {gantt_rows_str}
        ]);
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

@app.post("/preview")
def preview_map(request: PreviewRequest):
    locations = request.jobs
    if not locations:
        raise HTTPException(status_code=400, detail="Locations not found")

    center_lat = locations[0].lat
    center_lon = locations[0].lon
    m = folium.Map(location=[center_lat, center_lon], zoom_start=11)

    for loc in locations:
        if loc.id == 0:
            folium.Marker([loc.lat, loc.lon], tooltip="MAIN DEPOT", icon=folium.Icon(color="black", icon="home", prefix="fa")).add_to(m)
        else:
            folium.Marker([loc.lat, loc.lon], tooltip=f"Stop ID: {loc.id} | Load: {loc.demand} kg", icon=folium.Icon(color="blue", icon="info-sign")).add_to(m)

    return {"status": "success", "map_html": m.get_root().render()}

@app.post("/optimize")
def optimize(request: OptimizationRequest):
    locations = [{"lat": j.lat, "lon": j.lon, "id": j.id, "demand": j.demand, "time_start": j.time_start, "time_end": j.time_end} for j in request.jobs]
    try:
        distance_matrix, duration_matrix = get_osrm_matrices(locations)
    except:
        raise HTTPException(status_code=500, detail="Map service error")

    manager = pywrapcp.RoutingIndexManager(len(locations), request.vehicle_count, 0)
    routing = pywrapcp.RoutingModel(manager)

    def distance_callback(from_index, to_index):
        from_node, to_node = manager.IndexToNode(from_index), manager.IndexToNode(to_index)
        if request.open_path and to_node == 0: return 0 
        return distance_matrix[from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    def demand_callback(from_index):
        from_node = manager.IndexToNode(from_index)
        return locations[from_node]["demand"]
    demand_callback_index = routing.RegisterUnaryTransitCallback(demand_callback)

    def time_callback(from_index, to_index):
        from_node, to_node = manager.IndexToNode(from_index), manager.IndexToNode(to_index)
        if from_node == to_node: return 0
        if request.open_path and to_node == 0: return 0 
        return duration_matrix[from_node][to_node] + request.service_time

    time_callback_index = routing.RegisterTransitCallback(time_callback)

    if request.use_capacity:
        actual_capacity = request.vehicle_capacity
    else:
        actual_capacity = 9999999 

    routing.AddDimensionWithVehicleCapacity(demand_callback_index, 0, [actual_capacity] * request.vehicle_count, True, "Capacity")

    max_stops = len(locations) + 1 

    def stop_count_callback(from_index):
        return 1
    stop_count_callback_index = routing.RegisterUnaryTransitCallback(stop_count_callback)
    routing.AddDimension(
        stop_count_callback_index,
        0,          
        max_stops,  
        True,       
        'StopCount'
    )

    routing.AddDimension(transit_callback_index, 0, 99999999, True, 'Distance')
    routing.GetDimensionOrDie('Distance').SetGlobalSpanCostCoefficient(0)
    
    routing.AddDimension(time_callback_index, 99999, 99999, False, 'Time')
    time_dimension = routing.GetDimensionOrDie('Time')

    for vehicle_id in range(request.vehicle_count):
        time_dimension.CumulVar(routing.Start(vehicle_id)).SetValue(request.route_start_time)

    for i in range(1, len(locations)): 
        index = manager.NodeToIndex(i)
        start_t, end_t = locations[i]["time_start"], locations[i]["time_end"]
        if start_t > end_t: start_t, end_t = end_t, start_t
        if start_t != request.route_start_time or end_t != 1080:
            time_dimension.CumulVar(index).SetMin(start_t)
            time_dimension.SetCumulVarSoftUpperBound(index, end_t, 100)

    # =========================================================================
    # YENİ: BİRİM SENKRONİZASYONU (Mesafe ve Zaman Katsayıları Kalibre Edildi)
    # =========================================================================
    if request.optimization_goal == "vehicles":
        routing.SetFixedCostOfAllVehicles(100000)
    else:
        stop_dimension = routing.GetDimensionOrDie('StopCount')
        vehicles_to_force = min(request.vehicle_count, len(locations) - 1)
        for vehicle_id in range(vehicles_to_force):
            stop_dimension.SetCumulVarSoftLowerBound(routing.End(vehicle_id), 2, 10000000)

        if request.optimization_goal == "distance":
            pass
            
        elif request.optimization_goal == "balance":
            # 1 durak adaletsizliği = 50 KM ceza (Güçlü Durak Kümelemesi)
            stop_dimension.SetGlobalSpanCostCoefficient(50000)
            
        elif request.optimization_goal == "makespan":
            # 1 dakika uzama = 500 Metre Ceza (Sıkı zaman optimizasyonu)
            time_dimension.SetSpanCostCoefficientForAllVehicles(500)
            # 1 dakika bitiş saati farkı = 1.5 KM Ceza (Kümelemeyi bozmayan esnek adalet)
            time_dimension.SetGlobalSpanCostCoefficient(1500)

    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.SAVINGS
    search_parameters.local_search_metaheuristic = routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
    search_parameters.time_limit.seconds = 35 

    solution = routing.SolveWithParameters(search_parameters)

    if solution:
        routes_json = []
        depot = request.jobs[0]

        for vehicle_id in range(request.vehicle_count):
            index = routing.Start(vehicle_id)
            path_stops = []
            vehicle_load = 0 

            while not routing.IsEnd(index):
                arrival_min = solution.Min(time_dimension.CumulVar(index)) 
                arr_str = f"{(arrival_min // 60) % 24:02d}:{arrival_min % 60:02d}"
                loc = locations[manager.IndexToNode(index)]
                vehicle_load += loc["demand"]  
                path_stops.append({"lat": loc["lat"], "lon": loc["lon"], "id": loc["id"], "demand": loc["demand"], "arrival_time": arr_str})
                index = solution.Value(routing.NextVar(index))

            if len(path_stops) <= 1:
                routes_json.append({"vehicle_id": vehicle_id + 1, "path": [], "geometry": [], "total_km": 0, "total_load": 0})
                continue

            if not request.open_path:
                arrival_min = solution.Min(time_dimension.CumulVar(index))
                arr_str = f"{(arrival_min // 60) % 24:02d}:{arrival_min % 60:02d}"
                path_stops.append({"lat": depot.lat, "lon": depot.lon, "id": depot.id, "demand": 0, "arrival_time": arr_str})
            
            geometry, true_distance = get_route_geometry(path_stops)
            formatted_path = [{"order": i + 1, "lat": s["lat"], "lon": s["lon"], "original_id": s["id"], "demand": s["demand"], "arrival_time": s["arrival_time"]} for i, s in enumerate(path_stops)]

            routes_json.append({
                "vehicle_id": vehicle_id + 1,
                "path": formatted_path,
                "geometry": geometry,
                "total_km": round(true_distance / 1000, 2),
                "total_load": vehicle_load
            })

        result_data = {"status": "success", "routes": routes_json}
        result_data["map_html"] = generate_dashboard_html(request.jobs, result_data, request.service_time)
        return result_data
    raise HTTPException(status_code=400, detail="Route not found!")

from main_v2 import app as app_v2
app.mount("/v2", app_v2)
