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
    if len(locations) <= 1:
        return [], 0

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
        return int(data['distance_matrix'][manager.IndexToNode(from_index)][manager.IndexToNode(to_index)] * 1000)

    transit = routing.RegisterTransitCallback(callback)
    routing.SetArcCostEvaluatorOfAllVehicles(transit)
    params = pywrapcp.DefaultRoutingSearchParameters()
    params.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC

    solution = routing.SolveWithParameters(params)
    if solution:
        routes = []
        total_cost = 0
        for v in range(num_vehicles):
            index = routing.Start(v)
            route = []
            route_distance_km = 0
            while not routing.IsEnd(index):
                current = index
                route.append(manager.IndexToNode(index))
                index = solution.Value(routing.NextVar(index))
                if not routing.IsEnd(index):
                    route_distance_km += data['distance_matrix'][manager.IndexToNode(current)][manager.IndexToNode(index)]
            route.append(manager.IndexToNode(index))
            cost = route_distance_km * cost_per_km
            total_cost += cost
            if len(route) > 2:
                routes.append((route, route_distance_km, cost))
        return routes, total_cost
    return None, 0

# ------------------ VRPTW ------------------
def solve_vrptw(locations, time_windows, num_vehicles, depots):
    matrix = [[int(haversine(locations[i], locations[j]) * 1000 / 8.33) for j in range(len(locations))] for i in range(len(locations))]
    starts = depots * (num_vehicles // len(depots)) + depots[:num_vehicles % len(depots)]

    data = {
        "time_matrix": matrix,
        "time_windows": time_windows,
        "num_vehicles": num_vehicles,
        "starts": starts,
        "ends": starts,
    }

    manager = pywrapcp.RoutingIndexManager(len(matrix), num_vehicles, starts, starts)
    routing = pywrapcp.RoutingModel(manager)

    def callback(from_index, to_index):
        return data['time_matrix'][manager.IndexToNode(from_index)][manager.IndexToNode(to_index)]

    transit = routing.RegisterTransitCallback(callback)
    routing.SetArcCostEvaluatorOfAllVehicles(transit)

    routing.AddDimension(
        transit,
        30,
        1440,
        False,
        "Time"
    )
    time_dim = routing.GetDimensionOrDie("Time")

    for idx, window in enumerate(data['time_windows']):
        index = manager.NodeToIndex(idx)
        time_dim.CumulVar(index).SetRange(window[0], window[1])

    for i in range(num_vehicles):
        routing.AddVariableMinimizedByFinalizer(time_dim.CumulVar(routing.Start(i)))
        routing.AddVariableMinimizedByFinalizer(time_dim.CumulVar(routing.End(i)))

    params = pywrapcp.DefaultRoutingSearchParameters()
    params.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC

    solution = routing.SolveWithParameters(params)
    if solution:
        routes = []
        for v in range(num_vehicles):
            index = routing.Start(v)
            route = []
            while not routing.IsEnd(index):
                t = time_dim.CumulVar(index)
                route.append(f"{manager.IndexToNode(index)}(T: {solution.Min(t)}-{solution.Max(t)})")
                index = solution.Value(routing.NextVar(index))
            t = time_dim.CumulVar(index)
            route.append(f"{manager.IndexToNode(index)}(T: {solution.Min(t)}-{solution.Max(t)})")
            if len(route) > 2:
                routes.append(route)
        return routes
    return None

# ------------------ INTERFAZ ------------------
st.title("Optimización de Rutas (VRP / VRPTW)")
tab = st.selectbox("Selecciona el tipo de problema", ["VRP", "VRPTW"])

if 'num_paradas' not in st.session_state:
    st.session_state.num_paradas = 5

st.session_state.num_paradas = st.number_input("Número de paradas", min_value=2, value=st.session_state.num_paradas, step=1)
n = st.session_state.num_paradas

with st.form("form"):
    locations = []
    time_windows = []
    for i in range(n):
        col1, col2 = st.columns(2)
        with col1:
            lat = st.number_input(f"Latitud {i}", key=f"lat_{i}", format="%.6f")
        with col2:
            lon = st.number_input(f"Longitud {i}", key=f"lon_{i}", format="%.6f")
        locations.append((lat, lon))
        if tab == "VRPTW":
            t1 = st.number_input(f"Inicio ventana {i}", key=f"tw1_{i}")
            t2 = st.number_input(f"Fin ventana {i}", key=f"tw2_{i}")
            time_windows.append((t1, t2))

    v = st.number_input("Número de vehículos", min_value=1, value=2)
    d = st.number_input("Número de depósitos (primeras paradas)", min_value=1, value=1)
    depots = list(range(int(d)))

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
    elif tab == "VRPTW":
        result = solve_vrptw(locations, time_windows, int(v), depots)
        if result:
            for i, route in enumerate(result):
                st.success(f"Vehículo {i + 1}: {' → '.join(route)}")
        else:
            st.error("No se encontró solución")
