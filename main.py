from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from app.api.v1 import router_routes

# Inicializar la aplicación FastAPI
app = FastAPI(
    title="PetWalk API",
    description="API para la optimización de rutas de paseo con restricción de tiempo y calidad de paseo.",
    version="1.0.0"
)

# Configurar CORS
# Va permitir que el frontend acceda a la API sin problemas de seguridad
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Incluir las rutas del enrutador
app.include_router(router_routes.router, prefix="/api/v1")

# Ruta raíz para verificar que la API está funcionando
@app.get("/")
def read_root():
    return {"message": "PetWalk API is running"}