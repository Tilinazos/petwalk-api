from app.core.config.config import WALKING_PACING
from typing import Literal

WalkSpeed = Literal["ligero", "acelerado"]

# Calcula el tiempo en minutos para recorrer una arista segun su longitud y el ritmo de caminata
def calculate_edge_time(length_meters: float, pace: WalkSpeed) -> float:
    speed = WALKING_PACING.get(pace, WALKING_PACING["ligero"])
    
    if speed <= 0:
        return float('inf')

    return length_meters / speed