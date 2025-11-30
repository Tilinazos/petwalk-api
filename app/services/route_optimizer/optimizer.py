# app/services/route_optimizer/optimizer.py
from typing import List, Tuple
import networkx as nx
import osmnx as ox
from math import radians, cos, sin, sqrt, atan2
import heapq # Necesario para la Cola de Prioridad (implementación manual de A*)

from app.models.api.OptimizationRequest import OptimizationRequest, WalkSpeed, Coordinate
from app.models.api.RouteResponse import RouteResponse
from app.core.data_access.graph_repository import get_graph, get_closest_node
from app.core.config.config import WALKING_PACING, DEFAULT_PLACE_NAME
from app.core.utils.geometry_utils import calculate_edge_time


def haversine(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Cálculo de distancia haversine (Distancia de círculo máximo en metros)"""
    R = 6371000
    phi1, phi2 = radians(lat1), radians(lat2)
    dphi = radians(lat2 - lat1)
    dlambda = radians(lon2 - lon1)
    a = sin(dphi / 2)**2 + cos(phi1) * cos(phi2) * sin(dlambda / 2)**2
    c = 2 * atan2(sqrt(a), sqrt(1 - a))
    return R * c


def heuristic_haversine(u: int, v: int, G: nx.MultiDiGraph) -> float:
    """
    Heurística (h(n)) para A*: calcula la distancia Haversine (en metros) 
    entre dos nodos del grafo.
    """
    u_data = G.nodes[u]
    v_data = G.nodes[v]
    # 'y' es latitud, 'x' es longitud en OSMnx/NetworkX
    return haversine(u_data['y'], u_data['x'], v_data['y'], v_data['x'])


def a_star_shortest_path(G: nx.MultiDiGraph, source: int, target: int, 
                         heuristic, weight: str = "length") -> List[int]:
    """
    IMPLEMENTACIÓN MANUAL del algoritmo A* (A-ESTRELLA) SIN LIBRERÍAS EXTERNAS (heapq).
    Utiliza una lista simple para simular la cola de prioridad.
    """
    # 1. Inicialización
    g_cost = {node: float('inf') for node in G.nodes}  # Costo real (g(n))
    g_cost[source] = 0
    
    f_cost = {node: float('inf') for node in G.nodes}  # Costo total estimado (F(n) = g + h)
    f_cost[source] = heuristic(source, target, G)
    
    predecessor = {node: None for node in G.nodes}
    
    # Lista de nodos por visitar (simulando la cola de prioridad)
    open_set = [source] 

    # 2. Bucle Principal
    while open_set:
        
        # --- Lógica de la Cola de Prioridad Manual (O(V) por iteración) ---
        # Encuentra el nodo 'u' con el mínimo f_cost
        u = min(open_set, key=lambda node: f_cost[node])
        
        # Lo remueve del conjunto abierto
        open_set.remove(u)
        # -------------------------------------------------------------------

        # Condición de éxito
        if u == target:
            # Reconstrucción del camino
            path = []
            current = u
            while current is not None:
                path.append(current)
                current = predecessor.get(current)
            return path[::-1] # Retorna el camino invertido

        # 3. Expansión y Relajación de vecinos
        for v in G.neighbors(u):
            
            # Encuentra el costo mínimo de la arista (u, v) para el peso especificado
            edge_data = G.get_edge_data(u, v)
            min_edge_cost = float('inf')
            for key in edge_data:
                cost = edge_data[key].get(weight, 1)
                min_edge_cost = min(min_edge_cost, cost)

            tentative_g_cost = g_cost[u] + min_edge_cost

            # Relajación: Si encontramos un camino más corto a v
            if tentative_g_cost < g_cost[v]:
                predecessor[v] = u
                g_cost[v] = tentative_g_cost
                
                # Calcular h_cost y f_cost
                h_cost_v = heuristic(v, target, G)
                f_cost[v] = g_cost[v] + h_cost_v
                
                # Agregar/Actualizar en el conjunto abierto
                if v not in open_set:
                    open_set.append(v)
                
    return None # No se encontró ruta


def _find_farthest_park_node(G: nx.MultiDiGraph, start_lat: float, start_lon: float, 
                             max_time_minutes: int, pace: WalkSpeed) -> List[Tuple[int, float]]:
    """
    Busca y devuelve una lista de parques candidatos, ordenados del más lejano 
    al más cercano (usando la distancia de camino real como proxy).
    """
    print("Buscando parques con optimización A* y filtro Haversine...")
    
    start_node = get_closest_node(start_lat, start_lon)
    
    # 1. Definir el límite de búsqueda (60% del tiempo total)
    velocidad_m_min = WALKING_PACING.get(pace, 78)
    tiempo_ida_min = max_time_minutes * 0.60
    distancia_max_m = velocidad_m_min * tiempo_ida_min
    
    print(f"Distancia máxima de camino (m): {distancia_max_m:.1f}")
    
    # 2. Búsqueda y Filtrado Heurístico (parques)
    tags = {"leisure": "park"}
    parques = ox.features.features_from_place(DEFAULT_PLACE_NAME, tags=tags)
    
    if parques.empty:
        raise ValueError("No se encontraron parques en la zona.")
    
    parques = parques.copy()
    parques["center"] = parques.geometry.centroid 
    
    parques_candidatos = []
    
    for idx, row in parques.iterrows():
        lat_p = row["center"].y
        lon_p = row["center"].x
        nodo_parque = ox.nearest_nodes(G, X=lon_p, Y=lat_p)
        
        dist_aerea = haversine(start_lat, start_lon, lat_p, lon_p)
        
        # Filtro inicial por Haversine
        if dist_aerea <= distancia_max_m: 
            parques_candidatos.append({"node": nodo_parque, "dist_aerea": dist_aerea})

    if not parques_candidatos:
        raise ValueError("No hay parques dentro del rango posible según el tiempo disponible.")
    
    # 3. Ordenar por distancia Haversine (el más lejano primero)
    parques_candidatos.sort(key=lambda x: x["dist_aerea"], reverse=True)

    # 4. Verificación con shortest_path_length y coleccionar todos los candidatos
    candidatos_validos = [] # Lista de (nodo_parque, dist_camino_ida)
    
    for candidato in parques_candidatos:
        nodo_parque = candidato["node"]
        
        try:
            # Usar shortest_path_length para verificar la distancia REAL del camino
            # NOTA: Esta función de NetworkX (shortest_path_length) se deja para
            # eficiencia, ya que la implementación de A* desde cero devuelve la RUTA, 
            # no solo la longitud, lo que sería más lento de calcular aquí.
            dist_camino_ida = nx.shortest_path_length(
                G, 
                source=start_node, 
                target=nodo_parque, 
                weight="length" 
            )
            
            # Condición: debe caber en el 60% de la distancia máxima
            if dist_camino_ida <= distancia_max_m:
                candidatos_validos.append((nodo_parque, dist_camino_ida))
            
        except nx.NetworkXNoPath:
            continue
            
    if not candidatos_validos:
        raise ValueError("No hay parques que sean alcanzables dentro del límite de distancia de ida.")
    
    # Ordenar por la distancia real de camino (más largo primero)
    candidatos_validos.sort(key=lambda x: x[1], reverse=True)
    
    # Devolver la lista ordenada de (nodo_parque, distancia_ida)
    return candidatos_validos


def _calculate_forward_route(G: nx.MultiDiGraph, start_node: int, park_node: int) -> List[int]:
    """
    Ruta de IDA con pesos dog-friendly, usando el algoritmo A* MANUAL.
    """
    # Asignar pesos dog-friendly
    for u, v, data in G.edges(data=True):
        highway = data.get("highway", "")
        length = data.get("length", 1)
        peso = length
        
        # Penalizar avenidas
        if highway in ["primary", "secondary", "trunk", "motorway"]:
            peso *= 2.5
        
        # Favorecer caminos peatonales
        if highway in ["footway", "path", "pedestrian"]:
            peso *= 0.5
        
        data["dog_weight"] = peso
    
    # USO DEL ALGORITMO A* MANUAL
    return a_star_shortest_path(
        G, 
        start_node, 
        park_node, 
        heuristic_haversine, 
        weight="dog_weight"
    )


def _calculate_return_route(G: nx.MultiDiGraph, ruta_ida: List[int], 
                            start_node: int, park_node: int, is_cycle: bool) -> List[int]:
    """
    Ruta de VUELTA diferente o igual a la ida, basada en el flag is_cycle.
    """
    if is_cycle is False:
        # Si no se desea un ciclo, se usa la misma ruta invertida.
        return ruta_ida[::-1]

    # --- Lógica de Ciclo (is_cycle = True) ---
    H = G.copy()
    
    # Penalizar aristas de la ruta de ida para forzar un camino distinto
    PENALTY_FACTOR = 10.0 
    edges_ida = list(zip(ruta_ida[:-1], ruta_ida[1:]))
    
    for u, v in edges_ida:
        # Penalizar aristas en el grafo de vuelta, usando el peso 'dog_weight'
        for key, data in H.get_edge_data(u, v, default={}).items():
            if "dog_weight" in data:
                data["dog_weight"] *= PENALTY_FACTOR
        
        # Penalizar el sentido contrario (si existe el camino de v a u)
        for key, data in H.get_edge_data(v, u, default={}).items():
            if "dog_weight" in data:
                data["dog_weight"] *= PENALTY_FACTOR
            
    # Intentar ruta alternativa
    try:
        # USO DEL ALGORITMO A* MANUAL
        ruta = a_star_shortest_path(
            H, 
            park_node, 
            start_node, 
            heuristic_haversine,
            weight="dog_weight"
        )
        return ruta
    except:
        # Si no se encuentra ruta distinta con la penalización, usar la misma ruta
        return ruta_ida[::-1]


def _get_route_metrics(G: nx.MultiDiGraph, route_nodes: List[int], pace: WalkSpeed) -> Tuple[float, float]:
    """Calcula distancia y tiempo de la ruta"""
    total_time_minutes = 0.0
    total_distance_km = 0.0
    
    for u, v in zip(route_nodes[:-1], route_nodes[1:]):
        edge_data = G.get_edge_data(u, v)
        
        if not edge_data:
            continue
        
        first_edge = next(iter(edge_data.values()))
        length_meters = first_edge.get('length', 0.0)
        
        total_distance_km += length_meters / 1000.0
        total_time_minutes += calculate_edge_time(length_meters, pace) 
    
    return total_time_minutes, total_distance_km


def optimize_route(request: OptimizationRequest) -> RouteResponse:
    """
    Función principal - Bucle de verificación de tiempo total
    """
    G = get_graph()
    
    # 1. Obtener la lista de parques candidatos ordenados (nodo, dist_ida)
    try:
        candidatos_ordenados = _find_farthest_park_node(
            G, request.start_lat, request.start_lon, 
            request.max_time_minutes, request.walking_pace
        )
    except ValueError as e:
        # Retornar error si no se encuentra ni un solo candidato válido
        return RouteResponse(
            route=[], total_quality=0, total_time_minutes=0, 
            distance_km=0, message=str(e)
        )
    
    start_node = get_closest_node(request.start_lat, request.start_lon)
    
    # 2. Iterar sobre los candidatos (del más lejano al más cercano) y encontrar el primero que cumple el tiempo total
    for park_node, dist_ida_proxy in candidatos_ordenados:
        
        # a. Ruta de ida (usa dog_weight y A* manual)
        ruta_ida = _calculate_forward_route(G, start_node, park_node)
        
        # b. Ruta de vuelta (usa dog_weight, A* manual y penalización/is_cycle)
        ruta_vuelta = _calculate_return_route(G, ruta_ida, start_node, park_node, request.is_cycle)
        
        # c. Combinar ruta completa
        full_route_nodes = ruta_ida + ruta_vuelta[1:]
        
        if full_route_nodes is None or len(full_route_nodes) <= 1:
            continue
        
        # d. Calcular métricas totales (tiempo y distancia)
        total_time_minutes, total_distance_km = _get_route_metrics(
            G, full_route_nodes, request.walking_pace
        )
        
        # e. Condición de ÉXITO: Si el tiempo total cumple el límite
        if total_time_minutes <= request.max_time_minutes:
            
            message = f"Ruta óptima encontrada. Tiempo total: {total_time_minutes:.1f} min."
            
            # f. Convertir a coordenadas y retornar respuesta
            route_coordinates: List[Coordinate] = []
            try:
                for node_id in full_route_nodes:
                    node_data = G.nodes[node_id]
                    route_coordinates.append(Coordinate(lat=node_data['y'], lon=node_data['x']))
            except Exception as e:
                raise ValueError(f"Fallo al convertir coordenadas: {e}")
            
            return RouteResponse(
                route=route_coordinates,
                total_quality=len(route_coordinates),
                total_time_minutes=total_time_minutes,
                distance_km=total_distance_km,
                message=message
            )
            
    # 3. Si el bucle termina sin encontrar una ruta válida
    raise ValueError(f"Ningún parque encontrado permite un paseo completo dentro del límite de {request.max_time_minutes} minutos.")