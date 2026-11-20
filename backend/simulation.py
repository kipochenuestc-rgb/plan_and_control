"""Teaching demo: linearized lateral MPC + independent speed PI.

State [lateral error, heading error, actual steering]; input steering command.
Not a port of Autoware. SI units throughout; rear-axle reference point.
"""
import time
import numpy as np
import osqp
from scipy import sparse

DT, N, L, TAU = 0.1, 25, 2.8, 0.3
STEER_MAX, RATE_MAX = np.deg2rad(30), np.deg2rad(45)


def wrap(a):
    return np.arctan2(np.sin(a), np.cos(a))


class Simulation:
    def __init__(self, speed=6.0, offset=2.0):
        px = np.linspace(0, 260, 2601)
        py = 6 * np.sin(px / 25)
        dy = 6 / 25 * np.cos(px / 25)
        ddy = -6 / 625 * np.sin(px / 25)
        self.path = np.column_stack((px, py))
        self.arc = np.r_[0, np.cumsum(np.linalg.norm(np.diff(self.path, axis=0), axis=1))]
        self.heading = np.arctan(dy)
        self.curvature = ddy / (1 + dy * dy) ** 1.5
        psi = self.heading[0]
        self.x, self.y = -offset * np.sin(psi), offset * np.cos(psi)
        self.psi, self.v, self.delta = psi, 0.0, 0.0
        self.command, self.target, self.integral = 0.0, speed, 0.0
        self.t, self.index = 0.0, 0
        self.trace, self.history, self.prediction = [], [], []
        self.status, self.solve_ms = 'ready', 0.0
        self.failures, self.done, self.acceleration = 0, False, 0.0

    def reference(self, s):
        return np.array([np.interp(s, self.arc, self.path[:, 0]),
                         np.interp(s, self.arc, self.path[:, 1]),
                         np.interp(s, self.arc, self.heading),
                         np.interp(s, self.arc, self.curvature)]).T

    def match(self):
        # Local search prevents accidental jumps on paths near themselves.
        lo, hi = max(0, self.index - 10), min(len(self.path), self.index + 200)
        self.index = lo + int(np.argmin(np.sum((self.path[lo:hi] - [self.x, self.y])**2, axis=1)))
        r = self.reference(self.arc[self.index])
        ey = -(self.x-r[0])*np.sin(r[2]) + (self.y-r[1])*np.cos(r[2])
        return r, float(ey), float(wrap(self.psi-r[2]))

    def mpc(self, ey, epsi):
        # Speed is a given parameter, NOT an optimization variable.
        v = self.v
        refs = self.reference(self.arc[self.index] + np.arange(N+1)*v*DT)
        S = np.zeros((3, N))
        z = np.array([ey, epsi, self.delta])
        matrices, offsets = [], []
        uref = np.arctan(L*refs[:-1, 3])
        for i in range(N):
            dref = uref[i]
            gain = v/L / np.cos(dref)**2
            A = np.array([[1, v*DT, 0], [0, 1, DT*gain], [0, 0, 1-DT/TAU]])
            B = np.array([0, 0, DT/TAU])
            c = np.array([0, DT*(v/L*np.tan(dref)-gain*dref-v*refs[i, 3]), 0])
            S, z = A@S, A@z+c
            S[:, i] += B
            matrices.append(S.copy())
            offsets.append(z.copy())
        G, b = np.vstack(matrices), np.concatenate(offsets)
        weights = np.tile([8., 5., 0.], N)
        weights[-3:] *= 3
        D = np.eye(N) - np.eye(N, k=-1)
        previous = np.zeros(N)
        previous[0] = self.command
        H = G.T@(weights[:, None]*G) + .5*np.eye(N) + 3*D.T@D
        q = 2*(G.T@(weights*b) - .5*uref - 3*D.T@previous)
        C = sparse.csc_matrix(np.vstack([np.eye(N), D]))
        low = np.r_[np.full(N, -STEER_MAX), previous-RATE_MAX*DT]
        high = np.r_[np.full(N, STEER_MAX), previous+RATE_MAX*DT]
        start = time.perf_counter()
        solver = osqp.OSQP()
        solver.setup(P=sparse.csc_matrix(2*H), q=q, A=C, l=low, u=high,
                     verbose=False, eps_abs=1e-5, eps_rel=1e-5, max_iter=3000)
        result = solver.solve()
        self.solve_ms = (time.perf_counter()-start)*1000
        self.status = result.info.status
        if result.info.status_val not in (1, 2) or result.x is None:
            self.failures += 1
            self.prediction = []
            return self.command  # Hold previous steering; longitudinal controller brakes.
        predicted = (G@result.x+b).reshape(N, 3)
        r = refs[1:]
        self.prediction = np.column_stack((r[:, 0]-predicted[:, 0]*np.sin(r[:, 2]),
                                          r[:, 1]+predicted[:, 0]*np.cos(r[:, 2]))).tolist()
        # Clip numerical tolerance at hard command limits.
        return float(np.clip(result.x[0], max(-STEER_MAX, self.command-RATE_MAX*DT),
                             min(STEER_MAX, self.command+RATE_MAX*DT)))

    def step(self):
        if self.done:
            return self.snapshot()
        _, ey, epsi = self.match()
        self.command = self.mpc(ey, epsi)
        remaining = max(0, self.arc[-1]-self.arc[self.index]-1.0)
        target = min(self.target, np.sqrt(2*1.2*remaining))
        if self.status not in ('solved', 'solved inaccurate'):
            target = 0
        error = target-self.v
        candidate = np.clip(self.integral+error*DT, -5, 5)
        raw = 1.5*error+.25*candidate
        self.acceleration = float(np.clip(raw, -3, 2))
        if -3 < raw < 2:
            self.integral = candidate
        # Nonlinear plant, substeps; distinct from the linear prediction model.
        for _ in range(10):
            h = DT/10
            self.x += h*self.v*np.cos(self.psi)
            self.y += h*self.v*np.sin(self.psi)
            self.psi = float(wrap(self.psi+h*self.v/L*np.tan(self.delta)))
            self.delta += h*(self.command-self.delta)/TAU
            self.v = max(0., self.v+h*self.acceleration)
        self.t += DT
        _, ey, epsi = self.match()
        self.done = remaining < .2 and self.v < .12
        self.trace.append([self.x, self.y])
        self.history.append([self.t, ey, self.v, self.delta, self.command])
        return self.snapshot()

    def snapshot(self):
        _, ey, epsi = self.match()
        return dict(t=self.t, x=self.x, y=self.y, yaw=self.psi, speed=self.v,
                    target_speed=self.target, steering=float(np.rad2deg(self.delta)),
                    command=float(np.rad2deg(self.command)), error=ey,
                    heading_error=float(np.rad2deg(epsi)), solve_ms=self.solve_ms,
                    status=self.status, failures=self.failures, done=bool(self.done),
                    prediction=self.prediction, trace=self.trace[-3000:], history=self.history[-300:])
