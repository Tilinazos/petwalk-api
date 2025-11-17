# --- CONFIGURACIÓN DE DATOS ---
GRAPH_DATA_PATH = "data/lima_graph.pkl"
DEFAULT_PLACE_NAME = ["Miraflores, Lima, Peru", "San Isidro, Lima, Peru", "Jesus Maria, Lima, Peru"]
MIN_NODES_REQUIRED = 1500


# --- CONFIGURACIÓN DE RITMO DE CAMINATA ---
WALKING_PACING = {
    "ligero": 75,  # 4.5 km/h
    "acelerado": 90, # 5.4 km/h
}

TAGS_GREEN = {
    'leisure': ['park', 'garden', 'nature_reserve', 'playground'], 
    'landuse': ['grass', 'forest', 'meadow', 'recreation_ground']
}