# app/core/data_access/graph_repository.py
import osmnx as ox
import networkx as nx
import os
import pickle
from app.core.config.config import GRAPH_DATA_PATH, DEFAULT_PLACE_NAME

_GRAPH = None 

def load_graph():
    global _GRAPH
    if _GRAPH is not None:
        return _GRAPH

    if os.path.exists(GRAPH_DATA_PATH):
        print("INFO: Grafo encontrado. Cargando desde pickle...")
        try:
            with open(GRAPH_DATA_PATH, 'rb') as f:
                _GRAPH = pickle.load(f)
            print(f"INFO: Grafo cargado con {len(_GRAPH.nodes)} nodos.")
            return _GRAPH
        except Exception as e:
            print(f"ERROR: Fallo al cargar el grafo desde pickle: {e}. Se intentará descargar.")
            
    print(f"INFO: Descargando y procesando grafo para: {DEFAULT_PLACE_NAME}...")
    
    try:
        # Descargar grafo simplificado (IGUAL QUE TU SCRIPT)
        G = ox.graph_from_place(DEFAULT_PLACE_NAME, network_type='walk', simplify=True)
        G = ox.distance.add_edge_lengths(G)
        
        # Guardar
        os.makedirs(os.path.dirname(GRAPH_DATA_PATH), exist_ok=True)
        with open(GRAPH_DATA_PATH, 'wb') as f:
            pickle.dump(G, f)
            
        _GRAPH = G
        print(f"INFO: Grafo creado y guardado con {len(_GRAPH.nodes)} nodos.")
        return _GRAPH
        
    except Exception as e:
        print(f"ERROR: Fallo crítico al descargar el grafo: {e}")
        raise ConnectionError("No se pudo inicializar el modelo de grafo.")

try:
    load_graph()
except ConnectionError:
    pass 

def get_graph() -> nx.MultiDiGraph:
    if _GRAPH is None:
        raise ConnectionError("El grafo no está disponible en la memoria del servidor.")
    return _GRAPH

def get_closest_node(lat: float, lon: float):
    graph = get_graph()
    return ox.nearest_nodes(graph, lon, lat)