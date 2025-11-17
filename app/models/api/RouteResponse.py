from pydantic import BaseModel, Field
from typing import List, Literal, Optional
from app.models.api.OptimizationRequest import Coordinate

# Modelo de salida para la respuesta de la ruta óptima
class RouteResponse(BaseModel):
    route: List[Coordinate] = Field(..., description="Lista de coordenadas que forman la ruta óptima.")
    total_quality: float = Field(..., description="Puntuación total de calidad S(P) de la ruta.")
    total_time_minutes: float = Field(..., description="Tiempo total T(P) real de la ruta encontrada.")
    distance_km: float = Field(..., description="Distancia total recorrida en kilómetros.")
    message: Optional[str] = Field(None, description="Mensaje de feedback o alerta.")