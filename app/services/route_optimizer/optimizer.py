# app/services/route_optimizer/optimizer.py
import networkx as nx
import sys
import random
from typing import List, Tuple, Optional, Set
# Importaciones de módulos internos
from app.core.config.config import WALKING_PACING 
from app.core.data_access.graph_repository import get_graph, get_closest_node
import osmnx as ox
from app.core.utils.geometry_utils import calculate_edge_time
from app.models.api.OptimizationRequest import OptimizationRequest
from app.models.api.RouteResponse import RouteResponse
from app.models.api.OptimizationRequest import Coordinate 

# --- CONSTANTES OPTIMIZADAS ---
TIME_TOLERANCE = 0.35  # Tolerancia del 35% en el tiempo (muy flexible)
MAX_ATTEMPTS = 300     # Número máximo de intentos de búsqueda
REVISIT_PENALTY = 1.5  # Penalización MUY suave por revisitar nodos
RETURN_HOME_THRESHOLD = 0.5  # A partir del 50% del tiempo, priorizar retorno
MIN_CYCLE_NODES = 20   # Mínimo de nodos para considerar un ciclo válido


def _calculate_distance_to_start(G: nx.MultiDiGraph, node: int, start_node: int) -> float:
    """Calcula la distancia euclidiana entre un nodo y el nodo de inicio"""
    try:
        node_data = G.nodes[node]
        start_data = G.nodes[start_node]
        
        # Manejar tanto coordenadas proyectadas (x, y) como geográficas (lat, lon)
        if 'x' in node_data and 'y' in start_data:
            dx = float(node_data['x']) - float(start_data['x'])
            dy = float(node_data['y']) - float(start_data['y'])
        else:
            # Fallback a lat/lon si no hay x/y
            dx = (float(node_data.get('lon', 0)) - float(start_data.get('lon', 0))) * 111000
            dy = (float(node_data.get('lat', 0)) - float(start_data.get('lat', 0))) * 111000
        
        return (dx**2 + dy**2)**0.5
    except Exception as e:
        return float('inf')


def _get_weighted_neighbor(
    G: nx.MultiDiGraph, 
    current_node: int, 
    start_node: int,
    visited_nodes: Set[int],
    previous_node: Optional[int],
    time_progress: float
) -> Optional[int]:
    """
    Selecciona el siguiente vecino usando una estrategia heurística:
    - Al inicio: exploración más aleatoria
    - Después del 50% del tiempo: prioriza acercarse al inicio
    - Siempre permite revisitar nodos (con penalización suave)
    """
    
    neighbors = []
    weights = []
    
    for _, neighbor, edge_key, edge_data in G.edges(current_node, keys=True, data=True):
        # Evitar retroceso inmediato
        if previous_node and neighbor == previous_node:
            continue
            
        # Si es el nodo de inicio y ya tenemos un camino razonable
        if neighbor == start_node and len(visited_nodes) > MIN_CYCLE_NODES:
            # Alta prioridad para cerrar el ciclo si estamos cerca del tiempo objetivo
            if time_progress > RETURN_HOME_THRESHOLD:
                # Dar muy alta prioridad al cierre
                neighbors.append(neighbor)
                weights.append(100.0)  # Peso muy alto
                continue
        
        neighbors.append(neighbor)
        
        # Calcular peso basado en:
        # 1. Distancia al inicio (importante después del threshold)
        # 2. Si ya fue visitado (penalización MUY suave)
        distance_to_start = _calculate_distance_to_start(G, neighbor, start_node)
        
        if time_progress > RETURN_HOME_THRESHOLD:
            # Priorizar cercanía al inicio - usar distancia inversa
            weight = 10.0 / (distance_to_start / 1000.0 + 0.1)  # Normalizar a km
        else:
            # Exploración más aleatoria al inicio
            weight = 1.0
            # Pequeño bonus para alejarse del inicio al principio
            if time_progress < 0.3:
                weight = (distance_to_start / 1000.0 + 0.1)
        
        # Penalizar nodos visitados MUY SUAVEMENTE
        if neighbor in visited_nodes and neighbor != start_node:
            weight *= (1.0 / REVISIT_PENALTY)
        
        weights.append(max(weight, 0.001))  # Evitar pesos negativos o cero
    
    if not neighbors:
        return None
    
    # Normalizar pesos
    total_weight = sum(weights)
    if total_weight > 0:
        weights = [w / total_weight for w in weights]
    else:
        weights = [1.0 / len(weights)] * len(weights)
    
    try:
        return random.choices(neighbors, weights=weights, k=1)[0]
    except:
        return random.choice(neighbors) if neighbors else None


def _build_route_iterative(
    G: nx.MultiDiGraph,
    start_node: int,
    request: OptimizationRequest,
    attempt: int
) -> Optional[Tuple[List[int], float]]:
    """
    Construye una ruta de forma iterativa usando búsqueda guiada.
    """
    
    path_nodes = [start_node]
    visited_nodes = {start_node}
    current_time = 0.0
    target_time = request.max_time_minutes
    
    max_steps = 2000  # Límite de pasos por intento (aumentado)
    step_count = 0
    
    # Variable para tracking de intentos de cierre
    attempts_to_close = 0
    max_close_attempts = 50
    
    while step_count < max_steps:
        step_count += 1
        current_node = path_nodes[-1]
        previous_node = path_nodes[-2] if len(path_nodes) > 1 else None
        
        # Calcular progreso de tiempo
        time_progress = current_time / target_time if target_time > 0 else 0
        
        # Verificar si podemos cerrar el ciclo
        if len(path_nodes) > MIN_CYCLE_NODES and current_node == start_node:
            time_min = target_time * (1 - TIME_TOLERANCE)
            time_max = target_time * (1 + TIME_TOLERANCE)
            
            if time_min <= current_time <= time_max:
                return (path_nodes, current_time)
            elif current_time < time_min:
                # Continuar explorando si el tiempo es muy corto (pero con límite)
                attempts_to_close += 1
                if attempts_to_close > max_close_attempts:
                    return None  # Demasiados intentos de cierre prematuro
            else:
                # Tiempo excedido, este intento falló
                return None
        
        # Verificar si excedemos el tiempo máximo
        if current_time > target_time * (1 + TIME_TOLERANCE):
            return None
        
        # Estrategia especial: si estamos cerca del tiempo objetivo y cerca del inicio, intentar cerrar
        if time_progress > 0.7 and len(path_nodes) > MIN_CYCLE_NODES:
            # Verificar si hay camino directo al inicio
            if G.has_edge(current_node, start_node):
                edge_data = G.get_edge_data(current_node, start_node, key=0)
                edge_time = calculate_edge_time(edge_data.get('length', 0), request.walking_pace)
                projected_time = current_time + edge_time
                
                time_min = target_time * (1 - TIME_TOLERANCE)
                time_max = target_time * (1 + TIME_TOLERANCE)
                
                if time_min <= projected_time <= time_max:
                    # ¡Cierre perfecto encontrado!
                    path_nodes.append(start_node)
                    return (path_nodes, projected_time)
        
        # Seleccionar siguiente vecino
        next_node = _get_weighted_neighbor(
            G, current_node, start_node, visited_nodes, previous_node, time_progress
        )
        
        if next_node is None:
            # Sin vecinos disponibles, este intento falló
            return None
        
        # Calcular tiempo de la arista
        edge_data = G.get_edge_data(current_node, next_node, key=0)
        if not edge_data:
            return None
            
        edge_length = edge_data.get('length', 0)
        edge_time = calculate_edge_time(edge_length, request.walking_pace)
        
        # Aplicar penalización si revisitamos (muy suave ahora)
        if next_node in visited_nodes and next_node != start_node:
            edge_time *= REVISIT_PENALTY
        
        # Actualizar estado
        path_nodes.append(next_node)
        visited_nodes.add(next_node)
        current_time += edge_time
    
    # Alcanzamos el límite de pasos sin encontrar solución
    return None


def _get_walking_pace_value(pace_str: str) -> float:
    """
    Convierte el string de walking_pace a un valor numérico en m/s
    """
    pace_map = {
        "lento": 1.0,      # 3.6 km/h
        "ligero": 1.2,     # 4.3 km/h  
        "normal": 1.2,     # 4.3 km/h
        "rapido": 1.4,     # 5.0 km/h
        "muy_rapido": 1.6  # 5.8 km/h
    }
    
    pace_lower = pace_str.lower() if isinstance(pace_str, str) else "ligero"
    return pace_map.get(pace_lower, 1.2)  # Default: ligero


def optimize_route(request: OptimizationRequest) -> RouteResponse:
    """
    Función principal para optimizar rutas circulares.
    Utiliza búsqueda iterativa con múltiples intentos aleatorios.
    """
    
    # Convertir walking_pace si viene como string
    if isinstance(request.walking_pace, str):
        walking_pace_value = _get_walking_pace_value(request.walking_pace)
    else:
        walking_pace_value = float(request.walking_pace)
    
    print(f"\n🚶 Walking pace: {request.walking_pace} -> {walking_pace_value} m/s")
    
    # Obtener el grafo de la red vial
    G = get_graph()
    
    print(f"\n📍 Buscando nodo inicial cerca de ({request.start_lat}, {request.start_lon})")
    print(f"📊 Grafo tiene {len(G.nodes)} nodos y {len(G.edges)} aristas")
    
    # Verificar si el grafo está proyectado o en coordenadas geográficas
    sample_node = list(G.nodes(data=True))[0]
    is_projected = abs(sample_node[1].get('x', 0)) > 180 or abs(sample_node[1].get('y', 0)) > 90
    print(f"📐 Grafo está {'PROYECTADO' if is_projected else 'en coordenadas GEOGRÁFICAS'}")
    if is_projected:
        sample_coords = (sample_node[1].get('y'), sample_node[1].get('x'))
        print(f"   Ejemplo de coordenadas: {sample_coords}")
    
    # 1. Encontrar el nodo más cercano - SIEMPRE usar OSMnx que maneja ambos sistemas
    start_node_osmid = None
    try:
        # OSMnx maneja automáticamente coordenadas proyectadas y geográficas
        start_node_osmid = ox.nearest_nodes(G, request.start_lon, request.start_lat)
        print(f"✓ OSMnx encontró nodo: {start_node_osmid}")
    except Exception as e:
        print(f"✗ Error con OSMnx: {e}")
        
        # Método manual de respaldo
        print("⚠ Intentando búsqueda manual...")
        min_dist = float('inf')
        for node, data in G.nodes(data=True):
            try:
                if is_projected:
                    # El grafo está proyectado, necesitamos proyectar las coordenadas del request
                    # Por ahora, saltamos este nodo
                    continue
                else:
                    # Coordenadas geográficas
                    dist = ((data['y'] - request.start_lat)**2 + 
                           (data['x'] - request.start_lon)**2)**0.5
                    if dist < min_dist and G.degree(node) >= 2:
                        min_dist = dist
                        start_node_osmid = node
            except:
                continue
    
    if start_node_osmid is None or start_node_osmid not in G:
        raise ValueError(
            f"No se pudo encontrar un nodo válido cerca de ({request.start_lat}, {request.start_lon}). "
            f"Verifica que las coordenadas estén dentro del área del grafo cargado."
        )

    # Verificar que el nodo tenga conexiones
    node_degree = G.degree(start_node_osmid)
    node_coords = (G.nodes[start_node_osmid].get('y'), G.nodes[start_node_osmid].get('x'))
    
    print(f"✓ Nodo encontrado: {start_node_osmid}")
    print(f"  Coordenadas del nodo: {node_coords}")
    print(f"  Grado (conexiones): {node_degree}")
    
    if node_degree == 0:
        print(f"⚠ Nodo sin conexiones, buscando alternativa...")
        
        # Buscar nodos cercanos con BUENA conectividad (priorizar nodos con muchas conexiones)
        nearby_nodes = []
        for node in G.nodes():
            degree = G.degree(node)
            if degree >= 3:  # Al menos 3 conexiones para mejor flexibilidad
                try:
                    dist = _calculate_distance_to_start(G, node, start_node_osmid)
                    if dist < float('inf') and dist < 5000:  # Dentro de 5km en coordenadas proyectadas
                        nearby_nodes.append((node, dist, degree))
                except:
                    continue
        
        if not nearby_nodes:
            # Relajar requisito a 2 conexiones si no encontramos nada
            print("⚠ No se encontraron nodos con 3+ conexiones, buscando con 2+ conexiones...")
            for node in G.nodes():
                degree = G.degree(node)
                if degree >= 2:
                    try:
                        dist = _calculate_distance_to_start(G, node, start_node_osmid)
                        if dist < float('inf') and dist < 10000:
                            nearby_nodes.append((node, dist, degree))
                    except:
                        continue
        
        if not nearby_nodes:
            raise ValueError(
                f"No se encontraron calles bien conectadas cerca del punto ({request.start_lat}, {request.start_lon}). "
                f"El grafo puede no cubrir esta zona adecuadamente."
            )
        
        # Ordenar por conectividad (más conexiones primero) y luego por distancia
        nearby_nodes.sort(key=lambda x: (-x[2], x[1]))
        start_node_osmid = nearby_nodes[0][0]
        new_coords = (G.nodes[start_node_osmid].get('y'), G.nodes[start_node_osmid].get('x'))
        
        print(f"✓ Usando nodo alternativo {start_node_osmid}")
        print(f"  Coordenadas: {new_coords}")
        print(f"  Conexiones: {nearby_nodes[0][2]}")
        print(f"  Distancia al punto original: {nearby_nodes[0][1]:.0f}m (proyectadas)")
        
        node_degree = nearby_nodes[0][2]
    
    # Verificar que el nodo tenga suficientes conexiones para hacer un ciclo
    if node_degree < 2:
        raise ValueError(
            f"El nodo más cercano ({start_node_osmid}) no tiene suficientes conexiones "
            f"para crear una ruta circular. Prueba con otra ubicación."
        )
    
    # Advertencia si la conectividad es baja
    if node_degree == 2:
        print(f"⚠ ADVERTENCIA: El nodo solo tiene 2 conexiones. Será difícil crear ciclos largos.")
        print(f"   Considera usar una ubicación más céntrica con más calles.")

    print(f"\n{'='*60}")
    print(f"Iniciando búsqueda de ruta circular")
    print(f"Nodo inicio: {start_node_osmid}")
    print(f"Ubicación: ({request.start_lat}, {request.start_lon})")
    print(f"Tiempo objetivo: {request.max_time_minutes} min (±{TIME_TOLERANCE*100}%)")
    print(f"Rango aceptable: {request.max_time_minutes*(1-TIME_TOLERANCE):.1f} - {request.max_time_minutes*(1+TIME_TOLERANCE):.1f} min")
    print(f"{'='*60}\n")
    
    # 2. Ejecutar múltiples intentos de búsqueda
    best_solution = None
    best_time_diff = float('inf')
    
    # Crear un request modificado con el pace numérico para las funciones internas
    class RequestWithNumericPace:
        def __init__(self, original_request, pace_value):
            self.start_lat = original_request.start_lat
            self.start_lon = original_request.start_lon
            self.max_time_minutes = original_request.max_time_minutes
            self.walking_pace = pace_value
            self.is_cycle = getattr(original_request, 'is_cycle', True)
    
    modified_request = RequestWithNumericPace(request, walking_pace_value)
    
    for attempt in range(MAX_ATTEMPTS):
        result = _build_route_iterative(G, start_node_osmid, modified_request, attempt)
        
        if result:
            path_nodes, total_time = result
            time_diff = abs(total_time - request.max_time_minutes)
            
            print(f"Intento {attempt + 1}/{MAX_ATTEMPTS}: ✓ Ruta encontrada - "
                  f"{len(path_nodes)} nodos, {total_time:.2f} min "
                  f"(diff: {time_diff:.2f} min)")
            
            # Verificar si es solución válida
            time_min = request.max_time_minutes * (1 - TIME_TOLERANCE)
            time_max = request.max_time_minutes * (1 + TIME_TOLERANCE)
            
            if time_min <= total_time <= time_max:
                print(f"\n🎉 ¡Solución óptima encontrada en intento {attempt + 1}!")
                best_solution = result
                break
            
            # Guardar como mejor solución si está más cerca
            if time_diff < best_time_diff:
                best_solution = result
                best_time_diff = time_diff
        else:
            if (attempt + 1) % 10 == 0:
                print(f"Intento {attempt + 1}/{MAX_ATTEMPTS}: ✗ Sin solución")
    
    # 3. Validar y Formatear la Respuesta
    if best_solution is None:
        raise ValueError(
            f"No se encontró una ruta circular después de {MAX_ATTEMPTS} intentos. "
            f"Punto: ({request.start_lat}, {request.start_lon}), "
            f"Tiempo: {request.max_time_minutes} min. "
            f"Sugerencias: (1) Aumenta el tiempo objetivo, "
            f"(2) Prueba otra ubicación más céntrica, "
            f"(3) Aumenta MAX_ATTEMPTS en optimizer.py"
        )

    best_path_nodes, total_time_minutes = best_solution
    
    print(f"\n{'='*60}")
    print(f"✓ Ruta final seleccionada:")
    print(f"  - Nodos: {len(best_path_nodes)}")
    print(f"  - Tiempo: {total_time_minutes:.2f} min")
    print(f"  - Es ciclo: {best_path_nodes[0] == best_path_nodes[-1]}")
    print(f"{'='*60}\n")
    
    # 4. Construir la respuesta con coordenadas y métricas
    route_coordinates = []
    total_distance_m = 0.0
    total_quality = 0.0 
    
    for i, node_osmid in enumerate(best_path_nodes):
        # Obtener coordenadas del nodo
        route_coordinates.append(Coordinate(
            lat=G.nodes[node_osmid]['y'],
            lon=G.nodes[node_osmid]['x']
        ))
        
        # Sumar distancia y calidad de las aristas
        if i > 0:
            u = best_path_nodes[i-1]
            v = node_osmid
            edge_data = G.get_edge_data(u, v, key=0)
            
            if edge_data and 'length' in edge_data:
                total_distance_m += edge_data['length']
                total_quality += edge_data.get('quality_score', 0)

    # 5. Devolver la respuesta final
    time_diff_pct = abs(total_time_minutes - request.max_time_minutes) / request.max_time_minutes * 100
    
    return RouteResponse(
        route=route_coordinates,
        total_quality=total_quality,
        total_time_minutes=total_time_minutes,
        distance_km=total_distance_m / 1000.0, 
        message=(
            f"Ruta circular encontrada usando búsqueda aleatoria guiada. "
            f"Tiempo: {total_time_minutes:.1f} min ({time_diff_pct:.1f}% diff objetivo), "
            f"Distancia: {total_distance_m/1000:.2f} km, "
            f"Pasos: {len(best_path_nodes)} nodos."
        )
    )