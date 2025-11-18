# app/core/data_access/graph_repository.py
import osmnx as ox
import networkx as nx
import os
import pickle
from app.core.config.config import GRAPH_DATA_PATH, DEFAULT_PLACE_NAME, TAGS_GREEN
import geopandas as gpd

# Variable global que almacena el grafo cargado en memoria
_GRAPH = None 


# Asignar pesos de calidad a las aristas del grafo
def _assign_quality_weights(G: nx.MultiDiGraph, green_geometries_proj):
    for u, v, k, data in G.edges(keys=True, data=True):
        
        # 1. Score Base (Tranquilidad vs. Ruido)
        highway_type = data.get('highway', ['unclassified'])
        if isinstance(highway_type, list): highway_type = highway_type[0]
            
        if 'residential' in highway_type or 'path' in highway_type or 'footway' in highway_type:
            score = 7.0
        elif 'primary' in highway_type or 'secondary' in highway_type or 'trunk' in highway_type:
            score = 3.0
        else:
            score = 5.0
            
        # 2. BONIFICACIÓN POR PROXIMIDAD A ÁREAS VERDES (Heurística Simplificada)
        
        # Para evitar complejidad, usaremos la heurística de longitud, y bonificamos si es una calle de paseo:
        if ('path' in highway_type or 'footway' in highway_type or 'living_street' in highway_type) and data['length'] < 100:
            score += 2.0  # Bonificación directa si es un camino de paseo corto
        
        data['quality_score'] = round(score, 1)


# Cargar el grafo desde el archivo pickle o descargar y procesar si no existe
def load_graph():
    global _GRAPH
    if _GRAPH is not None:
        return _GRAPH # Grafo ya en memoria

    if os.path.exists(GRAPH_DATA_PATH):
        print("INFO: Grafo encontrado. Cargando desde pickle...")
        try:
            with open(GRAPH_DATA_PATH, 'rb') as f:
                _GRAPH = pickle.load(f)
            print(f"INFO: Grafo cargado con {len(_GRAPH.nodes)} nodos.")
            return _GRAPH
        except Exception as e:
            print(f"ERROR: Fallo al cargar el grafo desde pickle: {e}. Se intentará descargar.")
            
    # --- Si no existe o falló la carga, se descarga y procesa ---
    print(f"INFO: Descargando y procesando grafo para: {DEFAULT_PLACE_NAME}...")
    
    try:
        # 0. DESCARGAR ÁREAS VERDES (En CRS geográfico)
        print("INFO: Descargando áreas verdes...")
        green_areas = ox.features_from_place(DEFAULT_PLACE_NAME, TAGS_GREEN)
        green_geometries_proj = green_areas['geometry'] 
        print(f"INFO: Se encontraron {len(green_geometries_proj)} áreas verdes.")

        # 1. DESCARGAR Y PROCESAR GRAFO VIAL
        # Grafo NO PROYECTADO y SIMPLIFICADO (Idéntico al script)
        G = ox.graph_from_place(DEFAULT_PLACE_NAME, network_type='walk', simplify=True)
        G = ox.distance.add_edge_lengths(G)

        # 2. ASIGNAR PESOS
        _assign_quality_weights(G, green_geometries_proj) 
        
        # 3. GUARDAR
        os.makedirs(os.path.dirname(GRAPH_DATA_PATH), exist_ok=True)
        with open(GRAPH_DATA_PATH, 'wb') as f:
            pickle.dump(G, f)
            
        _GRAPH = G
        print(f"INFO: Grafo creado, procesado y guardado con {len(_GRAPH.nodes)} nodos.")
        return _GRAPH
        
    except Exception as e:
        print(f"ERROR: Fallo crítico al descargar y procesar el grafo: {e}")
        raise ConnectionError("No se pudo inicializar el modelo de grafo.")

        
# Ejecuta la carga al importar el módulo para que esté listo al iniciar FastAPI
try:
    load_graph()
except ConnectionError:
    pass 

def get_graph() -> nx.MultiDiGraph:
    """Retorna el objeto NetworkX MultiDiGraph cargado."""
    if _GRAPH is None:
        raise ConnectionError("El grafo no está disponible en la memoria del servidor.")
    return _GRAPH

def get_closest_node(lat: float, lon: float):
    """Retorna el OSMID del nodo más cercano a las coordenadas dadas."""
    graph = get_graph()
    # Requiere scikit-learn
    return ox.nearest_nodes(graph, lon, lat)