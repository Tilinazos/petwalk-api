# app/services/route_optimizer/optimizer.py
from typing import List, Optional, Tuple
import networkx as nx
import osmnx as ox
from math import radians, cos, sin, sqrt, atan2
import geopandas as gpd 

from app.models.api.OptimizationRequest import OptimizationRequest, WalkSpeed, Coordinate
from app.models.api.RouteResponse import RouteResponse
from app.core.data_access.graph_repository import get_graph, get_closest_node
from app.core.config.config import WALKING_PACING, DEFAULT_PLACE_NAME
from app.core.utils.geometry_utils import calculate_edge_time


# --- 1. UTILIDADES DE GEOMETRÍA Y VELOCIDAD ---

def haversine(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    R = 6371000
    phi1, phi2 = radians(lat1), radians(lat2)
    dphi = radians(lat2 - lat1)
    dlambda = radians(lon2 - lon1)

    a = (sin(dphi / 2)**2 + cos(phi1) * cos(phi2) * sin(dlambda / 2)**2)
    c = 2 * atan2(sqrt(a), sqrt(1 - a))
    return R * c


def _get_walking_speed_mps(pace: WalkSpeed) -> float:
    speed_m_per_min = WALKING_PACING.get(pace, WALKING_PACING["ligero"])
    return speed_m_per_min / 60.0

# --- 2. LÓGICA DE BÚSQUEDA DE DESTINO (IDÉNTICA A TU SCRIPT) ---

def _find_farthest_park_node(G: nx.MultiDiGraph, start_lat: float, start_lon: float, max_time_minutes: int, pace: WalkSpeed) -> int:
    """
    Busca el nodo de parque más lejano dentro del 60% del tiempo máximo disponible.
    """
    
    start_node = get_closest_node(start_lat, start_lon)
    speed_mps = _get_walking_speed_mps(pace)

    # 1. Calcular distancia máxima permitida para la ida (LÓGICA DEL USUARIO: 60%)
    tiempo_ida_sec = max_time_minutes * 60 * 0.60
    distancia_max = speed_mps * tiempo_ida_sec

    # 2. Buscar áreas verdes
    place_name = DEFAULT_PLACE_NAME
    parques = ox.features.features_from_place(
        place_name, 
        tags={"leisure": "park"}
    )
        
    if parques.empty:
        raise ValueError("No se encontraron parques en la zona de búsqueda del grafo.")

    # 3. Calcular centroides de forma ROBUSTA (Solución al UserWarning/Crash)
    try:
        # 1. Proyectar temporalmente a un CRS local UTM 
        parques_proj, _ = ox.projection.project_gdf(parques, to_latlong=False)
        
        # 2. Calcular el centroide en el CRS proyectado (cálculo correcto)
        parques_proj["center"] = parques_proj.geometry.centroid 
        
        # 3. Reproyectar solo los centroides de vuelta a WGS 84 (lat/lon)
        parques_centers = parques_proj.set_geometry("center").to_crs(epsg=4326)
    except Exception as e:
        # Esto captura y limpia el error de GeoPandas/CRS que causaba el error 500
        raise ValueError(f"Error crítico de geometría al procesar áreas verdes: {e}")
    
    parques_validos = []  # (nodo_parque, dist_camino)

    # Iterar sobre los centroides reproyectados a lat/lon
    for _, row in parques_centers.iterrows():
        lat_p = row["center"].y 
        lon_p = row["center"].x 

        try:
            # G NO está proyectado, nearest_nodes espera lat/lon.
            park_node = ox.nearest_nodes(G, X=lon_p, Y=lat_p)

            # Calcular la distancia del camino más corto (peso='length')
            dist_camino = nx.shortest_path_length(
                G,
                source=start_node,
                target=park_node,
                weight="length"
            )

            # Guardar solo si está dentro del rango
            if dist_camino <= distancia_max:
                parques_validos.append((park_node, dist_camino))

        except nx.NetworkXNoPath:
            continue
        except Exception:
            continue

    if not parques_validos:
        raise ValueError("No hay un parque accesible que permita un paseo de ida y vuelta dentro del tiempo máximo.")

    # 4. Escoger el parque MÁS LEJANO
    parque_elegido = max(parques_validos, key=lambda x: x[1])
    return parque_elegido[0]


# --- 3. LÓGICA DE RUTA DE IDA Y VUELTA (sin cambios) ---

def _calculate_forward_route(G: nx.MultiDiGraph, start_node: int, park_node: int) -> List[int]:
    H = G.copy() 

    # Asignar pesos 'dog_weight'
    for u, v, k, data in H.edges(keys=True, data=True):
        
        highway_data = data.get("highway", "")
        if isinstance(highway_data, list): 
            highway = highway_data[0]
        else:
            highway = highway_data
            
        length = data.get("length", 1)
        peso = length 

        # Penalizar avenidas (ruido)
        if highway in ["primary", "secondary", "trunk", "motorway"]:
            peso *= 2.5

        # Favorecer caminos peatonales
        if highway in ["footway", "path", "pedestrian"]:
            peso *= 0.5

        data["dog_weight"] = peso

    # Calcular la ruta de ida
    try:
        ruta = nx.shortest_path(H, start_node, park_node, weight="dog_weight")
        
        # Copiar los 'dog_weight' al grafo principal G (para la ruta de vuelta)
        for u, v, k, data in G.edges(keys=True, data=True):
            if 'dog_weight' in H[u][v][k]:
                 G[u][v][k]['dog_weight'] = H[u][v][k]['dog_weight']

        return ruta
    except nx.NetworkXNoPath:
        # Fallback a la ruta más corta por longitud
        try:
            return nx.shortest_path(G, start_node, park_node, weight="length")
        except nx.NetworkXNoPath:
             return [start_node]


def _calculate_return_route(G: nx.MultiDiGraph, ruta_ida: List[int], start_node: int, park_node: int, is_cycle: bool) -> List[int]:
    
    if not is_cycle:
        return ruta_ida[::-1]
    
    H = G.copy()

    # Eliminar todas las aristas de la ruta de ida (en ambas direcciones)
    edges_ida = list(zip(ruta_ida[:-1], ruta_ida[1:]))
    for u, v in edges_ida:
        # Eliminar aristas de u a v
        if H.has_edge(u, v):
            keys = list(H[u][v].keys())
            for k in keys:
                H.remove_edge(u, v, key=k)

        # Eliminar aristas de v a u
        if H.has_edge(v, u):
            keys = list(H[v][u].keys())
            for k in keys:
                H.remove_edge(v, u, key=k)

    # Intentar encontrar el camino de vuelta (park -> start) con 'dog_weight'
    try:
        ruta = nx.shortest_path(H, park_node, start_node, weight="dog_weight")
        return ruta
    except nx.NetworkXNoPath:
        # Fallback a la ruta de ida invertida
        return ruta_ida[::-1]


def _get_route_metrics(G: nx.MultiDiGraph, route_nodes: List[int], pace: WalkSpeed) -> Tuple[float, float, float]:
    total_time_minutes = 0.0
    total_distance_km = 0.0
    total_quality = 0.0

    for u, v in zip(route_nodes[:-1], route_nodes[1:]):
        edge_data = G.get_edge_data(u, v)
        
        best_edge_metrics = None
        max_quality = -float('inf')
        
        if not edge_data:
            continue
            
        for _, data in edge_data.items():
            current_quality = data.get('quality_score', 0.0) 
            
            if current_quality >= max_quality:
                max_quality = current_quality
                best_edge_metrics = data

        if best_edge_metrics:
            length_meters = best_edge_metrics.get('length', 0.0)
            
            total_distance_km += length_meters / 1000.0
            total_time_minutes += calculate_edge_time(length_meters, pace)
            total_quality += max_quality 

    return total_time_minutes, total_distance_km, total_quality


# --- 4. FUNCIÓN PRINCIPAL DE SERVICIO ---

def optimize_route(request: OptimizationRequest) -> RouteResponse:
    
    G = get_graph()
    
    # 1. Buscar el nodo de parque de destino
    try:
        park_node = _find_farthest_park_node(
            G, 
            request.start_lat, 
            request.start_lon, 
            request.max_time_minutes, 
            request.walking_pace
        )
    except ValueError as e:
        raise ValueError(str(e))


    start_node = get_closest_node(request.start_lat, request.start_lon)

    # 2. Ruta de ida 
    ruta_ida = _calculate_forward_route(G, start_node, park_node)

    # 3. Ruta de vuelta 
    ruta_vuelta = _calculate_return_route(
        G, 
        ruta_ida, 
        start_node, 
        park_node, 
        request.is_cycle
    )

    # 4. Combinar la ruta completa 
    full_route_nodes = ruta_ida + ruta_vuelta[1:] 
        
    if len(full_route_nodes) <= 1:
         return RouteResponse(route=[], total_quality=0, total_time_minutes=0, distance_km=0, message="Ruta vacía o no válida.")


    # 5. Calcular las métricas totales de la ruta
    total_time_minutes, total_distance_km, total_quality = _get_route_metrics(
        G, 
        full_route_nodes, 
        request.walking_pace
    )

    # 6. ELIMINACIÓN DE LA VALIDACIÓN ESTRICTA DE TIEMPO (SOLICITUD DEL USUARIO)
    message = "Ruta de máxima calidad encontrada."
    if total_time_minutes > request.max_time_minutes:
        # Devuelve la ruta con una advertencia en lugar de un error.
        message = f"ADVERTENCIA: El tiempo calculado ({total_time_minutes:.1f} min) excede el máximo permitido ({request.max_time_minutes} min), pero la ruta fue retornada."


    # 7. Convertir los nodos OSM ID a coordenadas (lat/lon)
    route_coordinates: List[Coordinate] = []
    
    try:
        for node_id in full_route_nodes:
            node_data = G.nodes[node_id]
            coordinate = Coordinate(lat=node_data['y'], lon=node_data['x'])
            route_coordinates.append(coordinate)
            
    except Exception as e:
        raise ValueError(f"Fallo al convertir coordenadas de la ruta: {e}")


    # 8. Retornar el objeto de respuesta
    return RouteResponse(
        route=route_coordinates,
        total_quality=total_quality,
        total_time_minutes=total_time_minutes,
        distance_km=total_distance_km,
        message=message
    )