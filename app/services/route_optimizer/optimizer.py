# app/services/route_optimizer/optimizer.py
from typing import List, Tuple
import networkx as nx
import osmnx as ox
from math import radians, cos, sin, sqrt, atan2

from app.models.api.OptimizationRequest import OptimizationRequest, WalkSpeed, Coordinate
from app.models.api.RouteResponse import RouteResponse
from app.core.data_access.graph_repository import get_graph, get_closest_node
from app.core.config.config import WALKING_PACING, DEFAULT_PLACE_NAME
from app.core.utils.geometry_utils import calculate_edge_time


def haversine(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Cálculo de distancia haversine (IGUAL QUE TU SCRIPT)"""
    R = 6371000
    phi1, phi2 = radians(lat1), radians(lat2)
    dphi = radians(lat2 - lat1)
    dlambda = radians(lon2 - lon1)
    a = sin(dphi / 2)**2 + cos(phi1) * cos(phi2) * sin(dlambda / 2)**2
    c = 2 * atan2(sqrt(a), sqrt(1 - a))
    return R * c


def _find_farthest_park_node(G: nx.MultiDiGraph, start_lat: float, start_lon: float, 
                             max_time_minutes: int, pace: WalkSpeed) -> int:
    """
    Busca el parque MÁS LEJANO dentro del 60% del tiempo disponible.
    LÓGICA IDÉNTICA A TU SCRIPT.
    """
    print("Buscando parques...")
    
    start_node = get_closest_node(start_lat, start_lon)
    
    # Calcular distancia máxima (60% del tiempo)
    velocidad = 1.3  # m/s (IGUAL QUE TU SCRIPT)
    tiempo_ida_seg = max_time_minutes * 60 * 0.60
    distancia_max = velocidad * tiempo_ida_seg
    
    print(f"Distancia máxima permitida (m): {distancia_max:.1f}")
    
    # Buscar parques
    tags = {"leisure": "park"}
    parques = ox.features.features_from_place(DEFAULT_PLACE_NAME, tags=tags)
    
    if parques.empty:
        raise ValueError("No se encontraron parques en la zona.")
    
    parques = parques.copy()
    parques["center"] = parques.geometry.centroid
    
    parques_validos = []
    
    for idx, row in parques.iterrows():
        lat_p = row["center"].y
        lon_p = row["center"].x
        nodo_parque = ox.nearest_nodes(G, X=lon_p, Y=lat_p)
        
        try:
            dist_camino = nx.shortest_path_length(
                G, source=start_node, target=nodo_parque, weight="length"
            )
            
            if dist_camino <= distancia_max:
                parques_validos.append((nodo_parque, dist_camino))
        except:
            continue
    
    if not parques_validos:
        raise ValueError("No hay parques dentro del rango posible según el tiempo disponible.")
    
    # Escoger el MÁS LEJANO (CLAVE DE TU ALGORITMO)
    parque_elegido = max(parques_validos, key=lambda x: x[1])
    nodo_objetivo, dist = parque_elegido
    
    print(f"Parque elegido: nodo {nodo_objetivo} a {dist:.1f} m (el máximo posible).")
    return nodo_objetivo


def _calculate_forward_route(G: nx.MultiDiGraph, start_node: int, park_node: int) -> List[int]:
    """
    Ruta de IDA con pesos dog-friendly.
    LÓGICA IDÉNTICA A TU SCRIPT.
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
    
    return nx.shortest_path(G, start_node, park_node, weight="dog_weight")


def _calculate_return_route(G: nx.MultiDiGraph, ruta_ida: List[int], 
                            start_node: int, park_node: int) -> List[int]:
    """
    Ruta de VUELTA diferente a la ida.
    LÓGICA IDÉNTICA A TU SCRIPT.
    """
    # Crear copia del grafo
    H = G.copy()
    
    # Eliminar aristas de la ruta de ida
    edges_ida = list(zip(ruta_ida[:-1], ruta_ida[1:]))
    for u, v in edges_ida:
        if H.has_edge(u, v):
            H.remove_edge(u, v)
        if H.has_edge(v, u):
            H.remove_edge(v, u)
    
    # Intentar ruta alternativa
    try:
        ruta = nx.shortest_path(H, park_node, start_node, weight="dog_weight")
        print("Ruta de vuelta (diferente) encontrada!")
        return ruta
    except:
        print("No se pudo encontrar ruta distinta. Usando la misma ruta.")
        return ruta_ida[::-1]


def _get_route_metrics(G: nx.MultiDiGraph, route_nodes: List[int], pace: WalkSpeed) -> Tuple[float, float]:
    """Calcula distancia y tiempo de la ruta"""
    total_time_minutes = 0.0
    total_distance_km = 0.0
    
    for u, v in zip(route_nodes[:-1], route_nodes[1:]):
        edge_data = G.get_edge_data(u, v)
        
        if not edge_data:
            continue
        
        # Tomar el primer edge (grafo simplificado)
        first_edge = next(iter(edge_data.values()))
        length_meters = first_edge.get('length', 0.0)
        
        total_distance_km += length_meters / 1000.0
        total_time_minutes += calculate_edge_time(length_meters, pace)
    
    return total_time_minutes, total_distance_km


def optimize_route(request: OptimizationRequest) -> RouteResponse:
    """
    Función principal - ALGORITMO IDÉNTICO A TU SCRIPT
    """
    G = get_graph()
    
    # 1. Buscar parque más lejano
    try:
        park_node = _find_farthest_park_node(
            G, request.start_lat, request.start_lon, 
            request.max_time_minutes, request.walking_pace
        )
    except ValueError as e:
        raise ValueError(str(e))
    
    start_node = get_closest_node(request.start_lat, request.start_lon)
    
    # 2. Ruta de ida
    ruta_ida = _calculate_forward_route(G, start_node, park_node)
    
    # 3. Ruta de vuelta
    ruta_vuelta = _calculate_return_route(G, ruta_ida, start_node, park_node)
    
    # 4. Combinar ruta completa
    full_route_nodes = ruta_ida + ruta_vuelta[1:]
    
    if len(full_route_nodes) <= 1:
        return RouteResponse(
            route=[], total_quality=0, total_time_minutes=0, 
            distance_km=0, message="Ruta vacía o no válida."
        )
    
    # 5. Calcular métricas
    total_time_minutes, total_distance_km = _get_route_metrics(
        G, full_route_nodes, request.walking_pace
    )
    
    # 6. Mensaje de advertencia si excede tiempo
    message = "Ruta óptima encontrada."
    if total_time_minutes > request.max_time_minutes:
        message = f"ADVERTENCIA: Tiempo calculado ({total_time_minutes:.1f} min) excede el máximo ({request.max_time_minutes} min)."
    
    # 7. Convertir a coordenadas
    route_coordinates: List[Coordinate] = []
    try:
        for node_id in full_route_nodes:
            node_data = G.nodes[node_id]
            route_coordinates.append(Coordinate(lat=node_data['y'], lon=node_data['x']))
    except Exception as e:
        raise ValueError(f"Fallo al convertir coordenadas: {e}")
    
    # 8. Retornar respuesta
    return RouteResponse(
        route=route_coordinates,
        total_quality=len(route_coordinates),  # Simplificado
        total_time_minutes=total_time_minutes,
        distance_km=total_distance_km,
        message=message
    )