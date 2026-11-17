"""Manuscript-defined 58 features; ordering is explicit reconstruction metadata.

The first 28 indices retain the inherited scalar ordering. The new descriptor
uses beta as the outer loop, eta as the inner loop, and [free, unknown] pairs.
The manuscript does not recover the historical descriptor serialization order.
"""
import math
import numpy as np

BETA_DEGREES = (-45.0, -22.5, 0.0, 22.5, 45.0)
ETA_DEGREES = (-20.0, 0.0, 20.0)


def state_features(initial, candidate, goal, config):
    """Return 28 normalized state/task features in the documented index order."""
    terminal = candidate.terminal
    gx, gy, gh = (float(v) for v in goal)
    dx, dy = terminal.x - initial.x, terminal.y - initial.y
    cg, sg = math.cos(initial.psi), math.sin(initial.psi)
    ct, st = math.cos(terminal.psi), math.sin(terminal.psi)
    tx, ty = gx - terminal.x, gy - terminal.y
    delta = terminal.psi - initial.psi
    result = np.array([
        terminal.V / 20.0, terminal.gamma / 0.35,
        terminal.phi / 0.61, terminal.alpha / 0.44,
        terminal.T / 20.0, terminal.yaw_rate,
        initial.V / 20.0, initial.gamma / 0.35,
        initial.phi / 0.61, initial.alpha / 0.44,
        initial.T / 20.0, initial.yaw_rate,
        (cg * dx + sg * dy) / 100.0,
        (-sg * dx + cg * dy) / 100.0,
        (terminal.h - initial.h) / 30.0,
        math.sin(delta), math.cos(delta),
        (ct * tx + st * ty) / 250.0,
        (-st * tx + ct * ty) / 250.0,
        (gh - terminal.h) / 50.0,
        (terminal.h - config.min_altitude) / 50.0,
        (config.max_altitude - terminal.h) / 50.0,
        config.H / 10.0, config.B / 10.0,
        0.0, 0.0, candidate.cost / 20.0,
        config.vertical_clearance / 10.0,
    ], dtype=np.float64)
    if not np.isfinite(result).all():
        raise ValueError("State/task features contain nonfinite values")
    return result


def observation_features(terminal, observed_map, config):
    """Return 15 direction pairs from 75 point-state queries, without rollouts.

    Probes are at k*terminal.V*B/5, k=1..5; elevations are relative to the
    horizontal plane, not the aircraft flight-path angle. Occupancy fractions
    are implied by 1-free-unknown. This descriptor is not a clearance test.
    """
    vals = [terminal.x, terminal.y, terminal.h, terminal.V, terminal.psi, config.B]
    if not np.isfinite(vals).all() or terminal.V <= 0 or config.B <= 0:
        raise ValueError("Probe state and horizon must be finite with positive speed/horizon")
    radius = terminal.V * config.B
    fractions = []
    for beta_deg in BETA_DEGREES:
        angle = terminal.psi + math.radians(beta_deg)
        for eta_deg in ETA_DEGREES:
            eta = math.radians(eta_deg)
            direction = np.array([math.cos(eta)*math.cos(angle),
                                  math.cos(eta)*math.sin(angle), math.sin(eta)])
            counts = {"free": 0, "unknown": 0, "occupied": 0}
            for k in range(1, 6):
                p = np.asarray(terminal.position) + (k * radius / 5.0) * direction
                state = observed_map.state(tuple(float(x) for x in p))
                if state not in counts:
                    raise ValueError("Map.state must return free, unknown, or occupied")
                counts[state] += 1
            fractions.extend([counts["free"] / 5.0, counts["unknown"] / 5.0])
    return np.asarray(fractions, dtype=np.float64)


def encode_features(initial, candidate, goal, observed_map, config):
    """Return the complete [28 scalar, 30 observation] encoding."""
    return np.concatenate((state_features(initial, candidate, goal, config),
                           observation_features(candidate.terminal, observed_map, config)))
