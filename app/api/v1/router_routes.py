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
    try:
        get_graph()
    except ConnectionError as e:
        raise HTTPException(status_code=503, detail="Servicio no disponible: " + str(e))

    try:
        route_result = optimize_route(request)
        
        if not route_result.route:
            raise HTTPException(status_code=404, detail="No se encontró una ruta que cumpla todos los requisitos del paseo.")
            
        return route_result
        
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e))
    except Exception as e:
        print(f"ERROR: Fallo durante la optimización: {e}")
        raise HTTPException(status_code=500, detail="Error interno del servidor durante el cálculo de la ruta.")