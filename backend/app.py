from pathlib import Path
from threading import Lock
from fastapi import FastAPI
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel, Field
from .simulation import Simulation

app = FastAPI(title='MPC bicycle demo')
lock = Lock()
simulation = Simulation()

class Settings(BaseModel):
    speed: float = Field(default=6, ge=0, le=12)
    offset: float = Field(default=2, ge=-4, le=4)

class Step(BaseModel):
    speed: float = Field(default=6, ge=0, le=12)

@app.get('/api/state')
def state():
    with lock:
        return dict(state=simulation.snapshot(), path=simulation.path[::5].tolist())

@app.post('/api/reset')
def reset(settings: Settings):
    global simulation
    with lock:
        simulation = Simulation(settings.speed, settings.offset)
        return dict(state=simulation.snapshot(), path=simulation.path[::5].tolist())

@app.post('/api/step')
def step(settings: Step):
    with lock:
        simulation.target = settings.speed
        return simulation.step()

app.mount('/', StaticFiles(directory=Path(__file__).resolve().parents[1]/'frontend', html=True), name='frontend')
