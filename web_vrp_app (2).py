import streamlit as st
from ortools.constraint_solver import routing_enums_pb2, pywrapcp
import math

# ------------------ FUNCIONES GENERALES ------------------
def haversine(coord1, coord2):
    R = 6371000
    lat1, lon1 = coord1
    lat2, lon2 = coord2
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)
    a = math.sin(dphi / 2)**2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2)**2
    return 2 * R * math.atan2(math.sqrt(a), math.sqrt(1 - a)) / 1000  # distancia en km

# ------------------ VRP ------------------
def solve_vrp(locations, num_vehicles, depots, cost_per_km):
    distance_matrix = [[haversine(a, b) for b in locations] for a in locations]
    starts = depots * (num_vehicles // len(depots)) + depots[:num_vehicles % len(depots)]
    data = {
        "distance_matrix": distance_matrix,
        "num_vehicles": num_vehicles,
        "starts": starts,
        "ends": starts,
    }

    manager = pywrapcp.RoutingIndexManager(len(distance_matrix), num_vehicles, starts, starts)
    routing = pywrapcp.RoutingModel(manager)

    def callback(from_index, to_index):
        return int(data['distance_matrix'][manager.IndexToNode(from_index)][manager.IndexToNode(to_index)] * 1000)  # en metros para mantener precisión

    transit = routing.RegisterTransitCallback(callback)
    routing.SetArcCostEvaluatorOfAllVehicles(transit)
    params = pywrapcp.DefaultRoutingSearchParameters()
    params.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC

    solution = routing.SolveWithParameters(params)
    if solution:
        routes = []
        total_cost = 0
        for v in range(num_vehicles):
            index, route = routing.Start(v), []
            route_distance_km = 0
            while not routing.IsEnd(index):
                current = index
                route.append(manager.IndexToNode(index))
                index = solution.Value(routing.NextVar(index))
                route_distance_km += data['distance_matrix'][manager.IndexToNode(current)][manager.IndexToNode(index)]
            route.append(manager.IndexToNode(index))
            cost = route_distance_km * cost_per_km
            total_cost += cost
            routes.append((route, route_distance_km, cost))
        return routes, total_cost
    return None, 0

# ------------------ CVRP ------------------
def solve_cvrp(locations, demands, capacities, depots):
    n = len(locations)
    matrix = [[int(haversine(locations[i], locations[j]) * 1000) for j in range(n)] for i in range(n)]
    starts = depots * (len(capacities) // len(depots)) + depots[:len(capacities) % len(depots)]
    data = {
        "distance_matrix": matrix,
        "demands": demands,
        "vehicle_capacities": capacities,
        "num_vehicles": len(capacities),
        "starts": starts,
        "ends": starts,
    }

    manager = pywrapcp.RoutingIndexManager(n, len(capacities), starts, starts)
    routing = pywrapcp.RoutingModel(manager)

    def callback(from_index, to_index):
        return data['distance_matrix'][manager.IndexToNode(from_index)][manager.IndexToNode(to_index)]

    transit = routing.RegisterTransitCallback(callback)
    routing.SetArcCostEvaluatorOfAllVehicles(transit)

    def demand_callback(index):
        return data['demands'][manager.IndexToNode(index)]

    demand_index = routing.RegisterUnaryTransitCallback(demand_callback)
    routing.AddDimensionWithVehicleCapacity(demand_index, 0, data['vehicle_capacities'], True, "Capacity")

    params = pywrapcp.DefaultRoutingSearchParameters()
    params.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    solution = routing.SolveWithParameters(params)

    if solution:
        routes = []
        for v in range(data['num_vehicles']):
            index, route = routing.Start(v), []
            while not routing.IsEnd(index):
                route.append(f"{manager.IndexToNode(index)}(Load: {solution.Value(routing.GetDimensionOrDie('Capacity').CumulVar(index))})")
                index = solution.Value(routing.NextVar(index))
            route.append(f"{manager.IndexToNode(index)}")
            routes.append(route)
        return routes
    return None

# ------------------ INTERFAZ ------------------
st.title("Optimización de Rutas (VRP / CVRP)")
tab = st.selectbox("Selecciona el tipo de problema", ["VRP", "CVRP"])

if 'num_paradas' not in st.session_state:
    st.session_state.num_paradas = 5

st.session_state.num_paradas = st.number_input("Número de paradas", min_value=2, value=st.session_state.num_paradas, step=1)
n = st.session_state.num_paradas

with st.form("form"):
    locations = []
    for i in range(n):
        col1, col2 = st.columns(2)
        with col1:
            lat = st.number_input(f"Latitud {i}", key=f"lat_{i}", format="%.6f")
        with col2:
            lon = st.number_input(f"Longitud {i}", key=f"lon_{i}", format="%.6f")
        locations.append((lat, lon))

    v = st.number_input("Número de vehículos", min_value=1, value=2)
    d = st.number_input("Número de depósitos (primeras paradas)", min_value=1, value=1)
    depots = list(range(int(d)))

    if tab == "CVRP":
        demands = [st.number_input(f"Demanda en parada {i}", min_value=0, value=1, key=f"dem_{i}") for i in range(n)]
        capacities = [st.number_input(f"Capacidad del vehículo {j}", min_value=1, value=10, key=f"cap_{j}") for j in range(int(v))]

    cost_per_km = st.number_input("Costo por kilómetro recorrido ($)", min_value=0.0, value=100.0)

    submitted = st.form_submit_button("Calcular Ruta")

if submitted:
    if tab == "VRP":
        result, total_cost = solve_vrp(locations, int(v), depots, cost_per_km)
        if result:
            for i, (route, dist, cost) in enumerate(result):
                st.success(f"Vehículo {i + 1}: {' → '.join(map(str, route))} | Distancia: {dist:.2f} km | Costo: ${cost:,.0f}")
            st.info(f"Costo total de transporte: ${total_cost:,.0f}")
        else:
            st.error("No se encontró solución")
    elif tab == "CVRP":
        result = solve_cvrp(locations, demands, capacities, depots)
        if result:
            for i, route in enumerate(result):
                st.success(f"Vehículo {i + 1}: {' → '.join(route)}")
        else:
            st.error("No se encontró solución")
