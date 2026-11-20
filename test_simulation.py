import unittest
import numpy as np
from fastapi.testclient import TestClient
from backend.app import app
from backend.simulation import Simulation, DT, STEER_MAX, RATE_MAX


class SimulationTests(unittest.TestCase):
    def test_tracking_and_constraints(self):
        for speed, offset in [(3, -3), (6, 2), (12, 4)]:
            with self.subTest(speed=speed):
                sim = Simulation(speed, offset)
                for _ in range(220):
                    sim.step()
                history = np.array(sim.history)
                self.assertEqual(sim.failures, 0)
                self.assertTrue(np.isfinite(history).all())
                self.assertLess(np.sqrt(np.mean(history[120:, 1]**2)), .15)
                self.assertLessEqual(np.abs(history[:, 4]).max(), STEER_MAX+1e-8)
                self.assertLessEqual(np.abs(np.diff(np.r_[0, history[:, 4]])).max(), RATE_MAX*DT+1e-8)

    def test_speed_stop_and_endpoint(self):
        sim = Simulation()
        for _ in range(80):
            sim.step()
        sim.target = 0
        for _ in range(120):
            sim.step()
        self.assertLess(sim.v, .03)
        self.assertGreaterEqual(sim.v, 0)
        sim = Simulation(12)
        for _ in range(600):
            sim.step()
        self.assertTrue(sim.done)
        self.assertLess(sim.v, .12)

    def test_api(self):
        with TestClient(app) as client:
            self.assertEqual(client.get('/').status_code, 200)
            self.assertEqual(client.get('/vendor/vue.global.prod.js').status_code, 200)
            response = client.post('/api/reset', json={'speed': 6, 'offset': 2})
            self.assertEqual(response.status_code, 200)
            self.assertAlmostEqual(response.json()['state']['error'], 2)
            state = client.post('/api/step', json={'speed': 6}).json()
            self.assertAlmostEqual(state['t'], .1)
            self.assertEqual(len(state['prediction']), 25)
            self.assertEqual(client.post('/api/step', json={'speed': -1}).status_code, 422)


if __name__ == '__main__':
    unittest.main()
