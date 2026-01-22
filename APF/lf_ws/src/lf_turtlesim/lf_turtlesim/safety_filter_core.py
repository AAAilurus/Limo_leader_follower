import math
from dataclasses import dataclass
from typing import List, Tuple, Optional


def wrap_to_pi(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


@dataclass
class Rect:
    xmin: float
    xmax: float
    ymin: float
    ymax: float


@dataclass
class SafetyParams:
    # CBF parameters
    d_safe: float = 0.45     # safety margin around rectangles and walls
    alpha: float = 2.0       # CBF class-K gain

    # APF repulsion
    k_rep: float = 2.0       # repulsive steering gain
    rep_range: float = 1.25  # start repulsion when distance < rep_range

    # Velocity limits
    v_min: float = -1.5
    v_max: float =  2.0
    w_min: float = -4.0
    w_max: float =  4.0


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def point_to_rect_signed_distance_and_grad(px: float, py: float, rect: Rect) -> Tuple[float, Tuple[float, float]]:
    """
    Signed distance from point to rectangle boundary:
      - Positive outside rectangle: Euclidean distance to nearest boundary
      - Negative inside rectangle: minus distance to nearest edge
    Also returns a (unit-ish) gradient direction pointing outward.
    """
    # Clamp point to rectangle to get closest point on/in rectangle
    cx = clamp(px, rect.xmin, rect.xmax)
    cy = clamp(py, rect.ymin, rect.ymax)

    dx = px - cx
    dy = py - cy
    outside = (px < rect.xmin) or (px > rect.xmax) or (py < rect.ymin) or (py > rect.ymax)

    if outside:
        dist = math.hypot(dx, dy)
        if dist < 1e-9:
            # On corner numerically; choose any outward direction
            return 0.0, (1.0, 0.0)
        return dist, (dx / dist, dy / dist)

    # Inside rectangle: distance to nearest edge (negative signed distance)
    d_left   = px - rect.xmin
    d_right  = rect.xmax - px
    d_bottom = py - rect.ymin
    d_top    = rect.ymax - py

    dmin = min(d_left, d_right, d_bottom, d_top)

    # Gradient points outward through the nearest edge
    if dmin == d_left:
        grad = (-1.0, 0.0)
    elif dmin == d_right:
        grad = (1.0, 0.0)
    elif dmin == d_bottom:
        grad = (0.0, -1.0)
    else:
        grad = (0.0, 1.0)

    return -dmin, grad


def wall_distance_and_grad(px: float, py: float, wall: str, x_min: float, x_max: float, y_min: float, y_max: float):
    """
    Signed distance to world boundary line (positive inside boundary, negative outside).
    For turtlesim default world: x,y roughly in [0, 11].
    """
    if wall == "left":
        d = px - x_min
        grad = (1.0, 0.0)
    elif wall == "right":
        d = x_max - px
        grad = (-1.0, 0.0)
    elif wall == "bottom":
        d = py - y_min
        grad = (0.0, 1.0)
    elif wall == "top":
        d = y_max - py
        grad = (0.0, -1.0)
    else:
        raise ValueError("Unknown wall")
    return d, grad


def smooth_weight(h: float, rep_range: float) -> float:
    """
    Smoothly increases repulsion as h -> 0.
    h = d - d_safe.
    repulsion acts when d < rep_range, i.e. h < (rep_range - d_safe).
    """
    # Map h to a [0,1] weight using a smoothstep-like curve
    # When h is large -> weight ~ 0, when h is small/negative -> weight -> 1
    h0 = rep_range  # "far"
    # Shift to use distance-to-obstacle (approx). Here we treat h itself as "distance beyond safe"
    # Use sigmoid-like weight:
    s = 1.0 / (1.0 + math.exp(4.0 * (h - 0.2)))  # tunable; 0.2 is softness
    return s


def apply_safety_filter_unicycle(
    px: float, py: float, theta: float,
    v_des: float, w_des: float,
    rects: List[Rect],
    world_bounds: Rect,
    params: SafetyParams
) -> Tuple[float, float]:
    """
    Safety filter:
      1) CBF-style projection on v to satisfy hdot + alpha*h >= 0 for the most critical constraint
      2) APF steering (smooth repulsive omega)
    """
    # Build constraints: rectangles + walls as "rect-like" signed distances
    constraints: List[Tuple[float, Tuple[float, float]]] = []

    # Rectangles
    for r in rects:
        d, grad = point_to_rect_signed_distance_and_grad(px, py, r)
        h = d - params.d_safe
        constraints.append((h, grad))

    # World boundaries (turtlesim world)
    # Keep within [xmin, xmax] and [ymin, ymax] with margin
    xmin, xmax, ymin, ymax = world_bounds.xmin, world_bounds.xmax, world_bounds.ymin, world_bounds.ymax
    for wall in ["left", "right", "bottom", "top"]:
        d, grad = wall_distance_and_grad(px, py, wall, xmin, xmax, ymin, ymax)
        h = d - params.d_safe
        constraints.append((h, grad))

    # Desired command clamping
    v = clamp(v_des, params.v_min, params.v_max)
    w = clamp(w_des, params.w_min, params.w_max)

    # Heading direction
    tx = math.cos(theta)
    ty = math.sin(theta)

    # --- (1) CBF projection on v: pick the most critical constraint (smallest h) ---
    # We enforce: (grad · t) * v >= -alpha * h
    # This is a 1D QP projection onto a half-space in v.
    h_min = float("inf")
    critical = None
    for (h, grad) in constraints:
        if h < h_min:
            h_min = h
            critical = (h, grad)

    if critical is not None:
        h, (gx, gy) = critical
        A = gx * tx + gy * ty
        b = -params.alpha * h

        # If A is close to 0, heading does not change h quickly; rely more on steering via APF.
        # Otherwise project v to satisfy inequality.
        if abs(A) > 1e-6:
            # Constraint: A*v >= b
            # If A>0 => v >= b/A ; if A<0 => v <= b/A
            v_bound = b / A
            if A > 0:
                v = max(v, v_bound)
            else:
                v = min(v, v_bound)

    # Keep v within limits after projection
    v = clamp(v, params.v_min, params.v_max)

    # --- (2) APF-style repulsive steering on omega (smooth) ---
    w_rep = 0.0
    for (h, (gx, gy)) in constraints:
        wt = smooth_weight(h, params.rep_range)
        # Determine turning direction based on cross product of heading and gradient
        # cross = t x grad = tx*gy - ty*gx
        cross = tx * gy - ty * gx
        w_rep += wt * math.tanh(3.0 * cross)

    w = w + params.k_rep * w_rep
    w = clamp(w, params.w_min, params.w_max)

    return v, w
