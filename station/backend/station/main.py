"""Punto de entrada: `uvicorn station.main:app --host 127.0.0.1 --port 8090`
(UN solo worker: la cola de corridas vive en memoria)."""
from station.app import create_app

app = create_app()
