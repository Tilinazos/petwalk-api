from pydantic import BaseModel, Field
from typing import List, Literal, Optional

# Definición de los ritmos permitidos para la caminata
WalkSpeed = Literal["ligero", "acelerado"]

# Modelo para representar la solicitud de optimización de ruta
class OptimizationRequest(BaseModel):
    start_lat: float = Field(..., description="Latitud del punto de inicio del paseo")
    start_lon: float = Field(..., description="Longitud del punto de inicio del paseo")
    max_time_minutes: int = Field(..., gt=0, description="Presupuesto máximo de tiempo en minutos")
    is_cycle: bool = Field(True, description="Indica si la ruta prefiere tener un camino diferente de regreso")
    walking_pace: WalkSpeed = Field("ligero", description="Ritmo de caminata")

# Modelo para representar una coordenada 
class Coordinate(BaseModel):
    lat: float
    lon: float