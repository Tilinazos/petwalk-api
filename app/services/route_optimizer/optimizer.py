# app/services/route_optimizer/optimizer.py
import networkx as nx
import sys
import random
from typing import List, Optional, Tuple, Set
# Importaciones de módulos internos
from app.core.config.config import WALKING_PACING
from app.core.data_access.graph_repository import get_graph, get_closest_node
from app.core.utils.geometry_utils import calculate_edge_time
from app.models.api.OptimizationRequest import OptimizationRequest
from app.models.api.RouteResponse import RouteResponse
from app.models.api.OptimizationRequest import Coordinate
import osmnx as ox
from pyproj import Transformer

# Constantes optimizadas
TIME_TOLERANCE = 0.25  # 25% de tolerancia
REVISIT_PENALTY = 2.0  # Penalización suave
MIN_CYCLE_NODES = 10   # Mínimo de nodos para ciclo válido
MAX_ITERATIONS = 3000  # Máximo de iteraciones por intento


def _find_best_start_node(G: nx.MultiDiGraph, lat: float, lon: float) -> Tuple[int, int]:
    """
    Encuentra el mejor nodo de inicio con buena conectividad.
    """
    print(f"\n🔍 Buscando nodo con buena conectividad cerca de ({lat}, {lon})...")
    
    # Detectar si el grafo está proyectado
    sample_node = list(G.nodes(data=True))[0]
    sample_x = sample_node[1].get('x', 0)
    sample_y = sample_node[1].get('y', 0)
    is_projected = abs(sample_x) > 180 or abs(sample_y) > 90
    
    print(f"   📐 Tipo de grafo: {'PROYECTADO' if is_projected else 'GEOGRÁFICO'}")
    
    search_x, search_y = lon, lat
    
    if is_projected:
        try:
            graph_crs = G.graph.get('crs', None)
            if graph_crs:
                transformer = Transformer.from_crs("EPSG:4326", graph_crs, always_xy=True)
                search_x, search_y = transformer.transform(lon, lat)
                print(f"   ✓ Coordenadas proyectadas: ({search_x:.2f}, {search_y:.2f})")
        except Exception as e:
            print(f"   ⚠️  Error al proyectar: {e}")
    
    # Búsqueda de candidatos
    candidates = []
    for node, data in G.nodes(data=True):
        degree = G.degree(node)
        if degree < 2:
            continue
        
        try:
            node_x = data.get('x', 0)
            node_y = data.get('y', 0)
            dist = ((node_x - search_x)**2 + (node_y - search_y)**2)**0.5
            
            max_dist = 2000 if is_projected else 0.02
            if dist < max_dist:
                candidates.append((node, dist, degree))
        except (KeyError, TypeError):
            continue
    
    if not candidates:
        raise ValueError(f"No se encontraron nodos cerca de ({lat}, {lon})")
    
    # Ordenar por conectividad y distancia
    candidates.sort(key=lambda x: (-x[2], x[1]))
    best_node, best_dist, best_degree = candidates[0]
    
    print(f"   ✅ Nodo seleccionado: {best_node}")
    print(f"      - Conexiones: {best_degree}")
    print(f"      - Distancia: {best_dist:.1f} {'m' if is_projected else '°'}")
    
    return best_node, best_degree


def _calculate_distance_to_start(G: nx.MultiDiGraph, node: int, start_node: int) -> float:
    """Calcula distancia euclidiana entre dos nodos"""
    try:
        n1 = G.nodes[node]
        n2 = G.nodes[start_node]
        dx = float(n1.get('x', 0)) - float(n2.get('x', 0))
        dy = float(n1.get('y', 0)) - float(n2.get('y', 0))
        return (dx**2 + dy**2)**0.5
    except:
        return float('inf')


def _build_route_iterative(
    G: nx.MultiDiGraph,
    start_node: int,
    tiempo_objetivo_segundos: float,
    tolerancia: float,
    velocidad_caminata: float,
    attempt: int
) -> Optional[Tuple[List[int], float]]:
    """
    Construye una ruta de forma ITERATIVA (no recursiva) usando exploración guiada.
    Similar al algoritmo del script que funciona.
    """
    
    path = [start_node]
    visited = {start_node}
    tiempo_actual = 0.0
    
    tiempo_min = tiempo_objetivo_segundos * (1 - tolerancia)
    tiempo_max = tiempo_objetivo_segundos * (1 + tolerancia)
    
    iterations = 0
    
    while iterations < MAX_ITERATIONS:
        iterations += 1
        current_node = path[-1]
        previous_node = path[-2] if len(path) > 1 else None
        
        # Calcular progreso
        progreso = tiempo_actual / tiempo_objetivo_segundos if tiempo_objetivo_segundos > 0 else 0
        
        # Verificar si podemos cerrar el ciclo
        if len(path) > MIN_CYCLE_NODES and current_node == start_node:
            if tiempo_min <= tiempo_actual <= tiempo_max:
                print(f"      ✓ Ciclo válido encontrado: {len(path)} nodos, {tiempo_actual/60:.1f} min")
                return (path, tiempo_actual)
            elif tiempo_actual < tiempo_min:
                # Demasiado corto, seguir explorando
                pass
            else:
                # Demasiado largo, falló
                return None
        
        # Si excedemos tiempo, falló
        if tiempo_actual > tiempo_max:
            return None
        
        # Obtener vecinos
        neighbors = list(G.neighbors(current_node))
        
        if not neighbors:
            return None
        
        # Filtrar retroceso inmediato
        if previous_node:
            neighbors = [n for n in neighbors if n != previous_node]
        
        if not neighbors:
            return None
        
        # ESTRATEGIA DE SELECCIÓN DE VECINO
        # - Si progreso < 50%: exploración aleatoria (alejarse del inicio)
        # - Si progreso >= 50%: priorizar regresar al inicio
        
        if progreso < 0.5:
            # Fase de exploración: selección más aleatoria
            # Pequeña preferencia por alejarse del inicio
            weights = []
            for n in neighbors:
                dist_to_start = _calculate_distance_to_start(G, n, start_node)
                # Premiar alejarse del inicio
                weight = 1.0 + (dist_to_start / 10000.0)  # Normalizar
                
                # Penalizar ligeramente nodos visitados
                if n in visited and n != start_node:
                    weight *= 0.5
                
                weights.append(max(weight, 0.1))
            
            # Normalizar
            total = sum(weights)
            if total > 0:
                weights = [w/total for w in weights]
            
            try:
                next_node = random.choices(neighbors, weights=weights, k=1)[0]
            except:
                next_node = random.choice(neighbors)
        else:
            # Fase de retorno: priorizar acercarse al inicio
            # Si el inicio es vecino y el tiempo está bien, ir al inicio
            if start_node in neighbors:
                edge_data = G.get_edge_data(current_node, start_node, key=0)
                if edge_data:
                    edge_time = edge_data['length'] / velocidad_caminata
                    tiempo_proyectado = tiempo_actual + edge_time
                    
                    if tiempo_min <= tiempo_proyectado <= tiempo_max:
                        # ¡Cerrar el ciclo ahora!
                        path.append(start_node)
                        return (path, tiempo_proyectado)
            
            # Seleccionar vecino más cercano al inicio
            best_neighbor = None
            best_dist = float('inf')
            
            for n in neighbors:
                dist = _calculate_distance_to_start(G, n, start_node)
                if dist < best_dist:
                    best_dist = dist
                    best_neighbor = n
            
            next_node = best_neighbor if best_neighbor else random.choice(neighbors)
        
        # Calcular costo de la arista
        edge_data = G.get_edge_data(current_node, next_node, key=0)
        if not edge_data or 'length' not in edge_data:
            return None
        
        edge_time = edge_data['length'] / velocidad_caminata
        
        # Penalización suave por revisitar
        if next_node in visited and next_node != start_node:
            edge_time *= REVISIT_PENALTY
        
        # Actualizar estado
        path.append(next_node)
        visited.add(next_node)
        tiempo_actual += edge_time
    
    # Máximo de iteraciones alcanzado
    return None


def optimize_route(request: OptimizationRequest) -> RouteResponse:
    """
    Orquestador principal con búsqueda iterativa.
    """
    print("\n" + "="*70)
    print("🐾 INICIANDO BÚSQUEDA DE PASEO PERFECTO")
    print("="*70)
    
    G = get_graph()
    print(f"📊 Grafo: {len(G.nodes)} nodos, {len(G.edges)} aristas")
    
    # Encontrar nodo de inicio
    start_node_osmid, node_degree = _find_best_start_node(
        G, request.start_lat, request.start_lon
    )
    
    print(f"\n📍 Nodo de inicio: {start_node_osmid} ({node_degree} conexiones)")
    
    # Parámetros
    tiempo_objetivo_segundos = request.max_time_minutes * 60
    pace_value = WALKING_PACING.get(request.walking_pace, WALKING_PACING["ligero"])
    velocidad_caminata = pace_value / 60.0
    
    print(f"\n⚙️  Configuración:")
    print(f"   - Tiempo: {request.max_time_minutes} min (±{TIME_TOLERANCE*100}%)")
    print(f"   - Rango: {request.max_time_minutes*(1-TIME_TOLERANCE):.0f}-{request.max_time_minutes*(1+TIME_TOLERANCE):.0f} min")
    print(f"   - Velocidad: {velocidad_caminata:.2f} m/s")
    
    print(f"\n🔍 Iniciando búsqueda iterativa...")
    
    # Múltiples intentos
    max_attempts = 20  # Más intentos porque es más rápido
    best_solution = None
    best_time_diff = float('inf')
    
    for attempt in range(max_attempts):
        # Ajustar tolerancia gradualmente
        tolerancia_actual = TIME_TOLERANCE + (attempt * 0.02)
        
        result = _build_route_iterative(
            G, 
            start_node_osmid, 
            tiempo_objetivo_segundos,
            tolerancia_actual,
            velocidad_caminata,
            attempt
        )
        
        if result:
            path, tiempo = result
            time_diff = abs(tiempo/60.0 - request.max_time_minutes)
            
            if (attempt + 1) % 5 == 0 or time_diff < 5:
                print(f"   Intento {attempt + 1}: ✓ {len(path)} nodos, {tiempo/60:.1f} min")
            
            # Verificar si es solución válida
            if time_diff < best_time_diff:
                best_solution = result
                best_time_diff = time_diff
                
                # Si está muy cerca, terminar
                if time_diff < 2:  # Menos de 2 min de diferencia
                    print(f"   ✅ Solución óptima en intento {attempt + 1}")
                    break
        else:
            if (attempt + 1) % 5 == 0:
                print(f"   Intento {attempt + 1}: ✗")
    
    # Validar resultado
    if not best_solution:
        raise ValueError(
            f"No se encontró ruta después de {max_attempts} intentos.\n\n"
            f"Sugerencias:\n"
            f"  1. Reduce el tiempo a 30-60 minutos\n"
            f"  2. Usa ubicaciones más céntricas\n"
            f"  3. Cambia a ritmo 'ligero'"
        )
    
    path, tiempo_total = best_solution
    
    print(f"\n✅ Ruta encontrada: {len(path)} nodos, {tiempo_total/60:.1f} min")
    
    # Calcular métricas
    route_coordinates = []
    total_distance = 0.0
    total_quality = 0.0
    
    for i, node_id in enumerate(path):
        node_data = G.nodes[node_id]
        
        route_coordinates.append(Coordinate(
            lat=node_data.get('lat', node_data.get('y', 0)),
            lon=node_data.get('lon', node_data.get('x', 0))
        ))
        
        if i > 0:
            edge_data = G.get_edge_data(path[i-1], node_id, key=0)
            if edge_data:
                total_distance += edge_data.get('length', 0)
                total_quality += edge_data.get('quality_score', 0)
    
    tiempo_minutos = tiempo_total / 60.0
    
    print(f"\n{'='*70}")
    print(f"✅ RUTA GENERADA")
    print(f"{'='*70}")
    print(f"📏 {total_distance/1000:.2f} km")
    print(f"⏱️  {tiempo_minutos:.1f} min")
    print(f"⭐ {total_quality:.1f} pts")
    print(f"{'='*70}\n")
    
    return RouteResponse(
        route=route_coordinates,
        total_quality=total_quality,
        total_time_minutes=tiempo_minutos,
        distance_km=total_distance / 1000.0,
        message=f"Ruta circular de {tiempo_minutos:.0f} min y {total_distance/1000:.1f} km generada."
    )