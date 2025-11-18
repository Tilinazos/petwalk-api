# app/core/config/config.py

# --- CONFIGURACIÓN DE DATOS ---
GRAPH_DATA_PATH = "data/miraflores_graph.pkl" # Simplificado
DEFAULT_PLACE_NAME = "Miraflores, Lima, Peru" # Simplificado a un solo lugar
MIN_NODES_REQUIRED = 1500


# --- CONFIGURACIÓN DE RITMO DE CAMINATA ---
WALKING_PACING = {
    "ligero": 78,  # 4.68 km/h (1.3 m/s)
    "acelerado": 90, # 5.4 km/h
}

TAGS_GREEN = {
    'leisure': ['park', 'garden', 'nature_reserve', 'playground'], 
    'landuse': ['grass', 'forest', 'meadow', 'recreation_ground']
}