# app/services/route_optimizer/optimizer.py
import networkx as nx
import sys
from functools import lru_cache # Se usará para la Programación Dinámica
from typing import List, Tuple
# Importaciones de módulos internos
from app.core.config.config import WALKING_PACING
from app.core.data_access.graph_repository import get_graph, get_closest_node
from app.core.utils.geometry_utils import calculate_edge_time
from app.models.api.OptimizationRequest import OptimizationRequest
from app.models.api.RouteResponse import RouteResponse

# Aumentar el límite de recursión para permitir búsquedas profundas en el grafo (Backtracking)
# Esto evita que Python falle por desbordamiento de pila durante búsquedas largas.
sys.setrecursionlimit(5000) 

def _recursive_search(
    G: nx.MultiDiGraph, 
    start_node: int, 
    request: OptimizationRequest, 
    best_result: dict, # Usamos un diccionario para pasar resultados por referencia
    current_path_nodes: List[int],
    current_time_spent: float, 
    current_quality_sum: float
):
    """
    Función recursiva de Backtracking para buscar la ruta de máxima calidad
    con restricción de tiempo, optimizada con Poda.
    """
    
    current_node = current_path_nodes[-1]
    
    # --- PODA 1: Poda por tiempo (Restricción dura) ---
    # Si el tiempo gastado excede el presupuesto del usuario, se poda la rama
    if current_time_spent > request.max_time_minutes:
        return 

    # --- PODA 2: Poda por longitud de camino (Heurística) ---
    # Se aumenta el límite a 1000 nodos para dar más margen a las rutas de 60-90 minutos
    if len(current_path_nodes) > 1000:
        return 

    # --- COMPROBACIÓN Y ACTUALIZACIÓN DE CICLO VÁLIDO ---
    # Un ciclo es válido si regresa al inicio Y ha recorrido más de un nodo
    if current_node == start_node and len(current_path_nodes) > 1:
        if current_quality_sum > best_result['quality']:
            # Actualizar el mejor resultado global encontrado hasta ahora
            best_result['quality'] = current_quality_sum
            best_result['path'] = list(current_path_nodes)
            best_result['time'] = current_time_spent
            
        # No se retorna, permitimos que siga buscando ciclos más largos con mejor calidad.
    
    # Explorar vecinos
    for _, neighbor, edge_key, edge_data in G.edges(current_node, keys=True, data=True):
        
        # Evitar retroceso inmediato (para rutas más diversas)
        if len(current_path_nodes) > 1 and neighbor == current_path_nodes[-2]:
            continue

        # 1. Calcular Costos de la Arista
        edge_length = edge_data.get('length', 0)
        edge_quality = edge_data.get('quality_score', 0)
        
        # Aplicar el ritmo de caminata del usuario (tiempo en minutos)
        edge_time = calculate_edge_time(edge_length, request.walking_pace)
        
        # 2. Penalización por Revisitar (Heurística de "Vuelta Diferente")
        time_penalty = 0.0
        quality_modifier = 0.0
        
        if neighbor in current_path_nodes and neighbor != start_node:
            # Penalizar el tiempo (hacer la arista 50% más "cara" si se repite)
            time_penalty = edge_time * 1.5 
            # Penalizar la calidad (descontar la mitad de la calidad para evitar repeticiones)
            quality_modifier = edge_quality * -0.5 
        
        new_time = current_time_spent + edge_time + time_penalty
        new_quality = current_quality_sum + edge_quality + quality_modifier

        # 3. PODA 3: Poda de Próxima Iteración (Prueba si el paso entra en el presupuesto)
        # Se usa una pequeña tolerancia de 5% para dar margen.
        if new_time <= request.max_time_minutes * 1.05: 
            
            _recursive_search(
                G, 
                start_node, 
                request, 
                best_result, 
                current_path_nodes + [neighbor], 
                new_time, 
                new_quality
            )


def optimize_route(request: OptimizationRequest) -> RouteResponse:
    """
    Orquestador del servicio. Encuentra el nodo, ejecuta la búsqueda Backtracking optimizada
    y formatea la respuesta.
    """
    G = get_graph()
    
    # 1. Encontrar el nodo más cercano
    # Se usan los nombres correctos definidos en el modelo
    start_node_osmid = get_closest_node(request.start_lat, request.start_lon)

    if start_node_osmid is None or start_node_osmid not in G:
        raise ValueError("No se pudo encontrar el punto de inicio en el mapa vial.")

    # 2. Inicializar el resultado global (pasado por referencia)
    best_result = {
        'quality': -1.0,
        'path': [],
        'time': 0.0,
    }

    # 3. Ejecutar la Búsqueda
    _recursive_search(
        G, 
        start_node_osmid, 
        request, 
        best_result, 
        current_path_nodes=[start_node_osmid], 
        current_time_spent=0.0, 
        current_quality_sum=0.0
    )
    
    # 4. Validar y Formatear la Respuesta
    if not best_result['path']:
        # Se lanza el error que el router captura como 400 Bad Request
        raise ValueError("No se encontró una ruta que cumpla los criterios de tiempo y ciclo. Intente con un tiempo mayor (ej. 90 minutos) o una ubicación diferente.")

    # Obtener coordenadas de los nodos
    route_coordinates = []
    total_distance_m = 0.0
    
    for i, node_osmid in enumerate(best_result['path']):
        # Mapear a coordenada
        route_coordinates.append(Coordinate(
            lat=G.nodes[node_osmid]['y'],
            lon=G.nodes[node_osmid]['x']
        ))
        
        # Sumar distancia (necesitamos la distancia entre el nodo actual y el anterior)
        if i > 0:
            u = best_result['path'][i-1]
            v = node_osmid
            # Usamos get_edge_data para obtener la longitud de la arista
            edge_data = G.get_edge_data(u, v, key=0)
            if edge_data and 'length' in edge_data:
                total_distance_m += edge_data['length']

    # Devolver la respuesta final
    return RouteResponse(
        route=route_coordinates,
        total_quality=best_result['quality'],
        total_time_minutes=best_result['time'],
        distance_km=total_distance_m / 1000.0, # Convertir a KM
        message="Ruta de Máxima Calidad encontrada exitosamente."
    )