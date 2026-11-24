"""
Physics core: circles + static AABBs.

Conventions:
  - normal (nx, ny) points from the AABB toward the circle
  - positions/velocities in pixels and pixels/second
  - time in seconds
"""
import math


class Body:
    def __init__(self, pos, vel=(0.0, 0.0), is_static=False,
                 restitution=0.5, color=(200, 200, 200)):
        self.pos = [float(pos[0]), float(pos[1])]
        self.vel = [float(vel[0]), float(vel[1])]
        self.is_static = is_static
        self.restitution = restitution
        self.inv_mass = 0.0 if is_static else 1.0
        self.fast = False
        self.color = color


class Circle(Body):
    def __init__(self, pos, vel=(0.0, 0.0), radius=10, **kw):
        super().__init__(pos, vel, **kw)
        self.radius = float(radius)


class AABB(Body):
    def __init__(self, pos, half=(10.0, 10.0), **kw):
        super().__init__(pos, (0.0, 0.0), is_static=True, **kw)
        self.half = (float(half[0]), float(half[1]))


# ---------- integration ----------

def integrate(body, dt):
    if body.is_static:
        return
    body.pos[0] += body.vel[0] * dt
    body.pos[1] += body.vel[1] * dt


# ---------- discrete overlap ----------

def circle_aabb_overlap(circle, box):
    """Return (overlap, nx, ny) — normal points from box to circle."""
    cx, cy = circle.pos
    bx, by = box.pos
    hx, hy = box.half
    qx = max(bx - hx, min(cx, bx + hx))
    qy = max(by - hy, min(cy, by + hy))
    dx, dy = cx - qx, cy - qy
    d2 = dx * dx + dy * dy
    r = circle.radius
    if d2 >= r * r:
        return False, 0.0, 0.0
    if d2 > 1e-12:
        d = math.sqrt(d2)
        return True, dx / d, dy / d
    # centre is inside the box: pick least-penetration axis
    left, right = cx - (bx - hx), (bx + hx) - cx
    top, bottom = cy - (by - hy), (by + hy) - cy
    m = min(left, right, top, bottom)
    if m == left:   return True, -1.0, 0.0
    if m == right:  return True,  1.0, 0.0
    if m == top:    return True,  0.0, -1.0
    return True, 0.0, 1.0


# ---------- CCD: swept circle vs static AABB ----------

def circle_aabb_ccd(circle, box, dt):
    """
    Slab method on the expanded AABB.
    Returns (t_fraction_of_dt, nx, ny) for the earliest contact inside [0, 1],
    or None if no contact happens during this step.
    """
    r = circle.radius
    bx, by = box.pos
    hx, hy = box.half
    min_x, max_x = bx - hx - r, bx + hx + r
    min_y, max_y = by - hy - r, by + hy + r

    px, py = circle.pos
    vx = circle.vel[0] * dt
    vy = circle.vel[1] * dt

    t_near, t_far = -math.inf, math.inf
    nx = ny = 0.0

    # x slab
    if abs(vx) < 1e-12:
        if px < min_x or px > max_x:
            return None
    else:
        t1 = (min_x - px) / vx
        t2 = (max_x - px) / vx
        if t1 > t2:
            t1, t2 = t2, t1
        if t1 > t_near:
            t_near = t1
            nx = -1.0 if vx > 0 else 1.0
            ny = 0.0
        if t2 < t_far:
            t_far = t2
        if t_near > t_far:
            return None

    # y slab
    if abs(vy) < 1e-12:
        if py < min_y or py > max_y:
            return None
    else:
        t1 = (min_y - py) / vy
        t2 = (max_y - py) / vy
        if t1 > t2:
            t1, t2 = t2, t1
        if t1 > t_near:
            t_near = t1
            nx = 0.0
            ny = -1.0 if vy > 0 else 1.0
        if t2 < t_far:
            t_far = t2
        if t_near > t_far:
            return None

    if t_near < 0.0 or t_near > 1.0:
        return None
    return t_near, nx, ny


# ---------- collision response ----------

def apply_impulse_circle_vs_static(circle, nx, ny):
    """Reflect the circle's velocity against a static surface (normal from box)."""
    vn = circle.vel[0] * nx + circle.vel[1] * ny
    if vn >= 0.0:
        return
    e = circle.restitution
    circle.vel[0] -= (1.0 + e) * vn * nx
    circle.vel[1] -= (1.0 + e) * vn * ny


def resolve_penetration(circle, box, nx, ny):
    """Push the circle just outside the box along the given normal."""
    cx, cy = circle.pos
    bx, by = box.pos
    hx, hy = box.half
    if abs(nx) >= abs(ny):
        edge = (bx - hx) if nx < 0 else (bx + hx)
        target = edge + nx * circle.radius
        pen = (cx - target) * nx
    else:
        edge = (by - hy) if ny < 0 else (by + hy)
        target = edge + ny * circle.radius
        pen = (cy - target) * ny
    if pen > 0.0:
        circle.pos[0] += nx * pen
        circle.pos[1] += ny * pen


# ---------- step functions ----------

def step_discrete(dynamics, statics, dt):
    """One naive step: integrate, then push out & reflect if overlapping."""
    for b in dynamics:
        integrate(b, dt)
        for s in statics:
            hit, nx, ny = circle_aabb_overlap(b, s)
            if hit:
                resolve_penetration(b, s, nx, ny)
                apply_impulse_circle_vs_static(b, nx, ny)


def step_ccd(dynamics, statics, dt):
    """
    CCD step: for each body, iteratively find the earliest contact inside
    [0, remaining], advance to it, reflect, then continue with the rest.
    """
    for b in dynamics:
        if b.is_static:
            continue
        remaining = dt
        for _ in range(8):  # cap iterations to avoid infinite loops
            if remaining < 1e-6:
                break
            earliest = None  # (t, nx, ny)
            for s in statics:
                res = circle_aabb_ccd(b, s, remaining)
                if res is not None:
                    t, nx, ny = res
                    if earliest is None or t < earliest[0]:
                        earliest = (t, nx, ny)
            if earliest is None:
                integrate(b, remaining)
                break
            t, nx, ny = earliest
            # move exactly to the contact point
            b.pos[0] += b.vel[0] * t * remaining
            b.pos[1] += b.vel[1] * t * remaining
            apply_impulse_circle_vs_static(b, nx, ny)
            # nudge slightly out so we don't re-detect t=0
            b.pos[0] += nx * 0.25
            b.pos[1] += ny * 0.25
            remaining -= t * remaining
            remaining -= 1e-4
            if remaining < 0.0:
                remaining = 0.0


def step_multirate(slow_dynamics, fast_dynamics, statics, dt_frame, ratio=4):
    """
    Fast objects get `ratio` sub-steps per frame (ratio * 60 Hz update rate).
    Slow objects get one step per frame.
    Collision response uses discrete overlap within each sub-step.
    """
    dt_fast = dt_frame / ratio
    for _ in range(ratio):
        for b in fast_dynamics:
            integrate(b, dt_fast)
            for s in statics:
                hit, nx, ny = circle_aabb_overlap(b, s)
                if hit:
                    resolve_penetration(b, s, nx, ny)
                    apply_impulse_circle_vs_static(b, nx, ny)
    for b in slow_dynamics:
        integrate(b, dt_frame)
        for s in statics:
            hit, nx, ny = circle_aabb_overlap(b, s)
            if hit:
                resolve_penetration(b, s, nx, ny)
                apply_impulse_circle_vs_static(b, nx, ny)