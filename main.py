from fastapi import FastAPI, HTTPException
from pydantic import BaseModel
from typing import List
from folium.features import DivIcon
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
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

class PreviewRequest(BaseModel):
    jobs: List[Job]

OSRM_BASE_URL = "http://router.project-osrm.org"
SERVICE_TIME = 10 

def get_osrm_matrices(locations):
    coordinates = [f"{loc['lon']},{loc['lat']}" for loc in locations]
    coord_string = ";".join(coordinates)
    url = f"{OSRM_BASE_URL}/table/v1/driving/{coord_string}?annotations=distance,duration"
    try:
        response = requests.get(url)
        if response.status_code != 200: raise Exception("OSRM Hatası")
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

def generate_map_html(jobs, result_json):
    if not result_json.get("routes"): return "<h1>Rota yok</h1>"
    center_lat, center_lon = jobs[0].lat, jobs[0].lon
    m = folium.Map(location=[center_lat, center_lon], zoom_start=11)
    colors = ["red", "blue", "green", "purple", "orange", "darkred", "cadetblue", "darkblue", "darkgreen", "cadetblue"]

    for i, route in enumerate(result_json["routes"]):
        path = route["path"]
        geometry = route.get("geometry", [])
        color = colors[i % len(colors)]
        total_load = route.get("total_load", 0) 
        if not path: continue

        if geometry:
            folium.PolyLine(
                geometry, color=color, weight=4, opacity=0.8,
                tooltip=f"Araç {route['vehicle_id']}: {route['total_km']} km | Yük: {total_load} kg"
            ).add_to(m)

        for stop in path:
            orj_id, lat, lon = stop["original_id"], stop["lat"], stop["lon"]
            order, demand, arrival = stop["order"], stop.get("demand", 0), stop.get("arrival_time", "")

            if orj_id == 0:
                if order == 1:
                    folium.Marker([lat, lon], popup="DEPO (Çıkış)", icon=folium.Icon(color="black", icon="home", prefix="fa")).add_to(m)
            else:
                display_num = order - 1
                folium.Marker(
                    [lat, lon],
                    icon=DivIcon(
                        icon_size=(30, 30), icon_anchor=(15, 15),
                        html=f'<div style="font-size: 11pt; font-weight: bold; color: {color}; background-color: white; border: 2px solid {color}; border-radius: 50%; width: 30px; height: 30px; text-align: center; line-height: 26px;">{display_num}</div>'
                    )
                ).add_to(m)
                folium.Marker([lat, lon], tooltip=f"Durak {display_num}: {demand} Kg | Saat: {arrival}", opacity=0).add_to(m)
    return m.get_root().render()

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
            folium.Marker([loc.lat, loc.lon], tooltip="MERKEZ DEPO", icon=folium.Icon(color="black", icon="home", prefix="fa")).add_to(m)
        else:
            folium.Marker([loc.lat, loc.lon], tooltip=f"Durak ID: {loc.id} | Yük: {loc.demand} kg", icon=folium.Icon(color="blue", icon="info-sign")).add_to(m)

    return {"status": "success", "map_html": m.get_root().render()}

@app.post("/optimize")
def optimize(request: OptimizationRequest):
    locations = [{"lat": j.lat, "lon": j.lon, "id": j.id, "demand": j.demand, "time_start": j.time_start, "time_end": j.time_end} for j in request.jobs]
    try:
        distance_matrix, duration_matrix = get_osrm_matrices(locations)
    except:
        raise HTTPException(status_code=500, detail="Harita servisi hatası")

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
        return duration_matrix[from_node][to_node] + SERVICE_TIME

    time_callback_index = routing.RegisterTransitCallback(time_callback)

    actual_capacity = request.vehicle_capacity if request.use_capacity else 9999999
    routing.AddDimensionWithVehicleCapacity(demand_callback_index, 0, [actual_capacity] * request.vehicle_count, True, "Capacity")
    
    routing.AddDimension(transit_callback_index, 0, 99999999, True, 'Distance')
    # SPAGETTİ ÇÖZÜMÜ 1: Katsayı 0 yapıldı (Kümelenmeyi bozmasın diye)
    routing.GetDimensionOrDie('Distance').SetGlobalSpanCostCoefficient(0) 
    
    routing.AddDimension(time_callback_index, 99999, 99999, False, 'Time')
    time_dimension = routing.GetDimensionOrDie('Time')

    for vehicle_id in range(request.vehicle_count):
        time_dimension.CumulVar(routing.Start(vehicle_id)).SetValue(480)

    for i in range(1, len(locations)): 
        index = manager.NodeToIndex(i)
        start_t, end_t = locations[i]["time_start"], locations[i]["time_end"]
        if start_t > end_t: start_t, end_t = end_t, start_t
        if start_t != 480 or end_t != 1080:
            time_dimension.CumulVar(index).SetMin(start_t)
            time_dimension.SetCumulVarSoftUpperBound(index, end_t, 100)

    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    # SPAGETTİ ÇÖZÜMÜ 2: Mükemmel Coğrafi Kümeleme
    search_parameters.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PARALLEL_CHEAPEST_INSERTION
    search_parameters.local_search_metaheuristic = routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
    # SPAGETTİ ÇÖZÜMÜ 3: Stres testi ve büyük veriler için 30 saniye
    search_parameters.time_limit.seconds = 30 

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

            # Açık rota değilse depoya dönüşü ekle
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
        result_data["map_html"] = generate_map_html(request.jobs, result_data)
        return result_data
    raise HTTPException(status_code=400, detail="Rota bulunamadı!")
