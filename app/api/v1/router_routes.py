from fastapi import APIRouter, HTTPException
from app.services.route_optimizer.optimizer import optimize_route
from app.core.data_access.graph_repository import get_graph 
from app.models.api.OptimizationRequest import OptimizationRequest
from app.models.api.RouteResponse import RouteResponse

router = APIRouter(
    prefix="/routes",
    tags=["Routes"],
)

@router.post("/optimize", response_model=RouteResponse, status_code=200)
async def get_optimized_route(request: OptimizationRequest):
    """
    Endpoint principal para calcular la ruta de máxima calidad restringida por tiempo.
    
    Recibe los parámetros de latitud/longitud, tiempo máximo y ritmo de caminata.
    Llama a la capa de servicios para ejecutar el algoritmo Backtracking con PD.
    """
    
    # Verificación de salud del grafo (opcional, pero útil)
    try:
        get_graph()
    except ConnectionError as e:
        raise HTTPException(status_code=503, detail="Servicio no disponible: " + str(e))

    try:
        # Llamar a la capa de servicios
        route_result = optimize_route(request)
        
        # Si el algoritmo se ejecuta pero no encuentra una ruta válida, lanza 404
        if not route_result.route:
            raise HTTPException(status_code=404, detail="No se encontró una ruta de máxima calidad que cumpla todos los criterios (tiempo, ciclo).")
            
        return route_result
        
    except ValueError as e:
        # Errores de validación de datos o de lógica de negocio (ej. ubicación inválida)
        raise HTTPException(status_code=400, detail=str(e))
    except Exception as e:
        # Errores inesperados durante la ejecución del algoritmo
        print(f"ERROR: Fallo durante la optimización: {e}")
        raise HTTPException(status_code=500, detail="Error interno del servidor durante el cálculo de la ruta.")