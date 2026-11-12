"""SI-unit dynamics and libraries reconstructed from main11.tex, Appendix A.

This is a new float64 reference implementation, not a reproduction of the old
float32 executable or its results. Library ordering and target compensation
follow the archived runtime header where the current manuscript is brief.
The inherited alpha-command denominator (0.03 + T/L) and cos(gamma_c) term
are deliberately retained; no alternative inversion is silently substituted.

Envelope checks are sampled. Observed-space checks cover each linear position
segment using the larger endpoint margin; this conservative choice is explicit
and does not claim continuous-time validation of the nonlinear flight dynamics.
"""
from dataclasses import fields
import math

from .types import State, Control, Config, Rollout

G = 9.81
K_AERO = 1.86
_STATE_FIELDS = tuple(f.name for f in fields(State))
_ODE_FIELDS = _STATE_FIELDS[:9]
_CONTROL_FIELDS = ('phi', 'alpha', 'T', 'duration')


def _finite_state(state):
    return all(math.isfinite(getattr(state, name)) for name in _STATE_FIELDS)


def _finite_control(control):
    return all(math.isfinite(getattr(control, name)) for name in _CONTROL_FIELDS)


def _clip(value, low, high):
    return max(low, min(value, high))


def _wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def _lift(alpha):
    return 0.03 * math.sin(alpha) + 0.028


def _drag(alpha):
    return 0.001 * alpha * alpha + 0.002


def derivative(state, control):
    """Return a State-shaped vector of nine ODE rates (yaw_rate slot is zero).

    ``yaw_rate`` is stored motion information for encoding/target compensation,
    not a tenth independent ODE. The heading derivative is returned in ``psi``.
    """
    if (not _finite_state(state) or not _finite_control(control) or
            state.V <= 0 or abs(math.cos(state.gamma)) < 1e-10):
        raise ValueError('Nonfinite state/control or singular flight dynamics')
    c_gamma = math.cos(state.gamma)
    normal = state.T * math.sin(state.alpha) + K_AERO * state.V**2 * _lift(state.alpha)
    return State(
        x=state.V * c_gamma * math.cos(state.psi),
        y=state.V * c_gamma * math.sin(state.psi),
        h=state.V * math.sin(state.gamma),
        V=state.T * math.cos(state.alpha) - K_AERO * state.V**2 * _drag(state.alpha) - G * math.sin(state.gamma),
        gamma=(normal * math.cos(state.phi) - G * c_gamma) / state.V,
        psi=normal * math.sin(state.phi) / (state.V * c_gamma),
        phi=2.0 * (control.phi - state.phi),
        alpha=control.alpha - state.alpha,
        T=10.0 * (control.T - state.T),
        yaw_rate=0.0,
    )


def _increment(state, rate, amount):
    values = {name: getattr(state, name) + amount * getattr(rate, name) for name in _ODE_FIELDS}
    return State(**values, yaw_rate=state.yaw_rate)


def step(state, control, dt, method='euler'):
    """Advance all nine variables together, preserving actuator continuity.

    Euler stores the pre-step heading derivative as the inherited implementation
    did. RK4 stores the endpoint heading derivative. Heading is wrapped only
    after the complete integration step. No actuator state is reset to command.
    """
    if not math.isfinite(dt) or dt <= 0:
        raise ValueError('Integration step must be finite and positive')
    if method not in ('euler', 'rk4'):
        raise ValueError('method must be euler or rk4')
    k1 = derivative(state, control)
    if method == 'euler':
        result = _increment(state, k1, dt)
        yaw_rate = k1.psi
    else:
        k2 = derivative(_increment(state, k1, dt / 2), control)
        k3 = derivative(_increment(state, k2, dt / 2), control)
        k4 = derivative(_increment(state, k3, dt), control)
        values = {name: getattr(state, name) + dt / 6 * (
            getattr(k1, name) + 2 * getattr(k2, name) +
            2 * getattr(k3, name) + getattr(k4, name)) for name in _ODE_FIELDS}
        result = State(**values, yaw_rate=state.yaw_rate)
        yaw_rate = derivative(result, control).psi
    return State(result.x, result.y, result.h, result.V, result.gamma,
                 _wrap(result.psi), result.phi, result.alpha, result.T, yaw_rate)


def trim_state(V=15.5, **positionkwargs):
    """Solve steady, level, unbanked flight; accepts x, y, h and psi overrides.

    The equilibrium is solved from the actual ODE, not from the approximate
    command inversion. Requested speeds without a root in the published attack
    interval are rejected rather than assigned an invented trim state.
    """
    if not math.isfinite(V) or V <= 0:
        raise ValueError('Trim airspeed must be positive')
    if set(positionkwargs) - {'x', 'y', 'h', 'psi'}:
        raise TypeError('Trim overrides are x, y, h and psi only')
    if not all(math.isfinite(v) for v in positionkwargs.values()):
        raise ValueError('Nonfinite trim position/heading')
    def balance(alpha):
        return K_AERO * V**2 * (_drag(alpha) * math.tan(alpha) + _lift(alpha)) - G
    lo, hi = math.radians(-20), math.radians(25)
    flo, fhi = balance(lo), balance(hi)
    if flo * fhi > 0:
        raise ValueError('No level trim in the published angle-of-attack interval')
    for _ in range(70):
        mid = (lo + hi) / 2
        if balance(mid) >= 0:
            hi = mid
        else:
            lo = mid
    alpha = (lo + hi) / 2
    thrust = K_AERO * V**2 * _drag(alpha) / math.cos(alpha)
    if thrust > 20:
        raise ValueError('Trim requires thrust beyond the published limit')
    return State(V=V, alpha=alpha, T=thrust, **positionkwargs)


def _target_command(state, distance, heading, height, duration, config):
    """Inherited target-shift heuristic followed by saturated command inversion."""
    if config.compensate_tau > 0.1:
        nu = state.yaw_rate * config.compensate_tau / 2
        travel = state.V * config.compensate_tau
        shifted_distance = math.sqrt(max(0.01, distance**2 + travel**2 -
                                        2 * distance * travel * math.cos(heading - nu)))
        cosine = _clip((travel - distance * math.cos(heading - nu)) / shifted_distance, -1, 1)
        angle = math.acos(cosine)
        if heading - nu < 0:
            angle = -angle
        heading = _clip(_wrap(math.pi - angle - nu), -config.heading_span, config.heading_span)
        distance = shifted_distance
    speed = max(config.min_speed, state.V)
    distance = max(0.1, distance)
    gain = 2 * speed / distance
    gamma_c = math.atan2(height - state.h, distance)
    gamma_dot = gain * (gamma_c - state.gamma)
    phi = _clip(math.atan2(2 * speed**2 * math.sin(heading) * math.cos(state.gamma)**2,
                          distance * (G * math.cos(state.gamma) + speed * gamma_dot)),
                -config.max_roll, config.max_roll)
    lift_scale = K_AERO * speed**2 * max(0.1, math.cos(phi))
    # These expressions intentionally reproduce the inherited command heuristic.
    a = (speed * gamma_dot + G * math.cos(gamma_c)) / lift_scale
    denominator = max(1e-5, 0.03 + state.T / lift_scale)
    alpha = _clip(math.asin(_clip((a - 0.028) / denominator, -1, 1)),
                  config.min_alpha, config.max_alpha)
    thrust = _clip((K_AERO * speed**2 * _drag(alpha) + G * math.sin(gamma_c)) /
                   max(0.1, math.cos(alpha)) + 2.2 * gain * (config.cruise_speed - speed),
                   0, config.max_thrust)
    return Control(phi, alpha, thrust, duration)


def generate_primary(state, goal, config):
    """Stable height-major, heading-minor library (at most 406 by default).

    Seven goal-centered heights precede level and +/-1/16, +/-1/8, +/-1/4
    climb-angle targets. Height duplicates within 1e-4 m are removed in order.
    Horizontal target direction belongs to the task cost, not this library.
    """
    if not _finite_state(state) or len(goal) != 3 or not all(math.isfinite(v) for v in goal):
        raise ValueError('Nonfinite state/goal or malformed goal')
    distance = config.H * config.cruise_speed
    low = max(config.min_altitude, state.h - distance * math.tan(config.max_gamma))
    high = min(config.max_altitude, state.h + distance * math.tan(config.max_gamma))
    if low > high:
        return []
    center = _clip(goal[2], low, high)
    heights = []
    def add(height):
        height = _clip(height, low, high)
        if not any(abs(height - old) <= 1e-4 for old in heights):
            heights.append(height)
    half = config.vertical_levels // 2
    for j in range(-half, half + 1):
        height = center if half == 0 else (center + (center - low) * j / half
                                          if j < 0 else center + (high - center) * j / half)
        add(height)
    if config.vertical_levels > 1:
        add(state.h)
        for fraction in (1 / 16, 1 / 8, 1 / 4):
            add(state.h + distance * math.tan(fraction * config.max_gamma))
            add(state.h - distance * math.tan(fraction * config.max_gamma))
    headings = [0.0] if config.headings == 1 else [
        -config.heading_span + i * 2 * config.heading_span / (config.headings - 1)
        for i in range(config.headings)]
    return [_target_command(state, distance, heading, height, config.H, config)
            for height in heights for heading in headings]


def generate_successors(terminal, config):
    """Exactly 15 controls: level/up/down x straight/left-half/right-half/left/right.

    Height clipping does not deduplicate this bank: its fixed order defines
    both first-witness selection and the auxiliary work target.
    """
    if not _finite_state(terminal):
        raise ValueError('Nonfinite terminal state')
    distance = config.B * max(config.min_speed, terminal.V)
    heights = [_clip(h, config.min_altitude, config.max_altitude) for h in (
        terminal.h, terminal.h + distance * math.tan(config.max_gamma),
        terminal.h - distance * math.tan(config.max_gamma))]
    return [_target_command(terminal, distance, fraction * config.heading_span,
                            height, config.B, config)
            for height in heights for fraction in (0.0, -0.5, 0.5, -1.0, 1.0)]


def _in_envelope(state, config):
    return (config.min_speed <= state.V <= config.max_speed and
            config.min_altitude <= state.h <= config.max_altitude and
            abs(state.gamma) <= config.max_gamma and abs(state.phi) <= config.max_roll and
            config.min_alpha <= state.alpha <= config.max_alpha and
            0 <= state.T <= config.max_thrust)


def _control_valid(control, config):
    return (_finite_control(control) and control.duration > 0 and
            abs(control.phi) <= config.max_roll and
            config.min_alpha <= control.alpha <= config.max_alpha and
            0 <= control.T <= config.max_thrust)


def rollout(state, control, config, observed_map=None, start_time=0,
            proposal=True, method='euler', dt=None):
    """Validate the complete finite rollout, stopping on the first failed check.

    ``start_time`` is the global prediction-horizon offset, not wall-clock time:
    pass H for a successor. Returned times use that same offset. The map may be
    None for dynamics-only diagnostics; that is not an observed-space witness.
    Explicit dt overrides config.dt (Euler) or config.execution_dt (RK4).
    """
    if method not in ('euler', 'rk4'):
        raise ValueError('method must be euler or rk4')
    delta = dt if dt is not None else (config.dt if method == 'euler' else config.execution_dt)
    if not math.isfinite(delta) or delta <= 0:
        raise ValueError('Integration step must be finite and positive')
    result = Rollout([state], [start_time], False, 'invalid_dynamics', 0)
    if (not math.isfinite(start_time) or start_time < 0 or
            not _finite_state(state) or not _control_valid(control, config)):
        return result
    if not _in_envelope(state, config):
        result.reason = 'envelope'
        return result
    def margin(time):
        return config.margin + (config.reserve * _clip((time - 0.2) / 0.3, 0, 1) if proposal else 0)
    if observed_map is not None and not observed_map.segment_free(state.position, state.position, margin(start_time)):
        result.reason = 'observed_space'
        return result
    # Avoid cumulative floating drift: integrate ceil(duration/dt) intervals,
    # shortening only the final one. An attempted step consumes one work unit.
    count = max(1, math.ceil(control.duration / delta - 1e-12))
    current = state
    elapsed = 0.0
    for index in range(count):
        target = control.duration if index == count - 1 else min((index + 1) * delta, control.duration)
        step_dt = target - elapsed
        result.steps += 1
        try:
            new = step(current, control, step_dt, method)
        except (ValueError, OverflowError, ZeroDivisionError):
            result.reason = 'invalid_dynamics'
            return result
        result.states.append(new)
        result.times.append(start_time + target)
        if not _finite_state(new):
            result.reason = 'invalid_dynamics'
            return result
        if not _in_envelope(new, config):
            result.reason = 'envelope'
            return result
        if observed_map is not None:
            swept_margin = max(margin(start_time + elapsed), margin(start_time + target))
            if not observed_map.segment_free(current.position, new.position, swept_margin):
                result.reason = 'observed_space'
                return result
        current = new
        elapsed = target
    result.valid = True
    result.reason = 'accepted'
    return result
