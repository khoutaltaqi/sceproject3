"""Lecture 11A prediction, 11B/12A updates, and 12B angle wrap."""
import numpy as np


def wrap(a):
    return np.arctan2(np.sin(a), np.cos(a))  # 12B.3


def motion(x, u, dt):
    px, py, th, v, w = x
    c, s = np.cos(th), np.sin(th)
    out = x + dt * np.array([v*c, v*s, w, u[0], u[1]])
    out[2] = wrap(out[2])
    F = np.eye(5)  # 11A, slides 13-15
    F[0, 2], F[0, 3] = -v*s*dt, c*dt
    F[1, 2], F[1, 3], F[2, 4] = v*c*dt, s*dt, dt
    G = np.zeros((5, 2))
    G[3, 0] = G[4, 1] = dt
    return out, F, G


def tag_model(x, landmark):
    dx, dy = np.asarray(landmark) - x[:2]
    q = max(dx*dx + dy*dy, 1e-10)
    d = np.sqrt(q)
    h = np.array([d, wrap(np.arctan2(dy, dx) - x[2])])  # 12A.1
    H = np.array([[-dx/d, -dy/d, 0, 0, 0],
                  [dy/q, -dx/q, -1, 0, 0]])  # 12A.2, 12A.3
    return h, H


def cubic(t, T, length):
    # Phase 2 cubic, now with zero speed at both ends.
    r = np.clip(t/T, 0, 1)
    return length*(3*r*r - 2*r*r*r), length*6*r*(1-r)/T


class EKF:
    def __init__(self, c):
        self.x = np.zeros(5)
        self.P = np.diag(np.array([0.02, 0.02, 0.03, 0.03, 0.05])**2)
        self.Ro = np.diag([c['odom_speed_std_mps']**2, c['odom_turn_std_radps']**2])

    def predict(self, u, dt):
        self.x, F, G = motion(self.x, u, dt)
        Q = G @ np.diag([0.15**2, 0.5**2]) @ G.T
        Q += np.diag(np.array([0.003, 0.003, 0.006, 0.01, 0.02])**2)*dt
        self.P = F @ self.P @ F.T + Q

    def update(self, z, h, H, R, angle=None, gate=None):
        before, covariance = self.x.copy(), self.P.copy()
        e = np.asarray(z) - h
        if angle is not None:
            e[angle] = wrap(e[angle])  # Wrap bearing only, never range.
        S = H @ self.P @ H.T + R
        nis = float(e @ np.linalg.solve(S, e))
        used = gate is None or nis <= gate
        K, correction = None, np.zeros(5)
        if used:
            K = np.linalg.solve(S, H @ self.P).T
            correction = K @ e
            self.x += correction
            self.x[2] = wrap(self.x[2])
            A = np.eye(5) - K @ H
            self.P = A @ self.P @ A.T + K @ R @ K.T  # Joseph covariance form.
            self.P = (self.P + self.P.T)/2
        self.last_update = {key: np.asarray(value).tolist() for key, value in {
            'z': z, 'h': h, 'H': H, 'R': R, 'innovation': e, 'S': S,
            'correction': correction, 'x_prior': before, 'x_post': self.x,
            'P_prior': covariance, 'P_post': self.P}.items()}
        self.last_update.update(accepted=bool(used), nis=nis, gate=gate,
                                K=None if K is None else K.tolist())
        return used, e, nis

    def odom(self, v, w):
        H = np.zeros((2, 5))
        H[0, 3] = H[1, 4] = 1  # 11B, slide 11
        return self.update([v, w], self.x[3:5], H, self.Ro)

    def tag(self, z, landmark, R):
        h, H = tag_model(self.x, landmark)
        return self.update(z, h, H, R, angle=1, gate=18.42)
