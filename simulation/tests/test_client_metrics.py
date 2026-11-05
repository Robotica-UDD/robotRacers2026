from fastapi.testclient import TestClient
from simulation.service.app import create_app
from simulation.service.runtime import RobotService

def test_real_renderer_measurements_validation(tmp_path):
    with TestClient(create_app(RobotService(tmp_path),background=False)) as client:
        assert client.post('/api/client-metrics',json={'render_fps':59.5,'total_frames':120,'visible':True}).status_code==200
        metrics=client.get('/api/health').json()['client_metrics']
        assert metrics['render_fps']==59.5 and metrics['total_frames']==120 and metrics['visible']
        assert client.post('/api/client-metrics',json={'render_fps':None}).status_code==400
        assert client.post('/api/client-metrics',json={'render_fps':500}).status_code==400
        assert client.post('/api/client-metrics',json={'render_fps':10,'total_frames':-1}).status_code==400
