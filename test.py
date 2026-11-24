"""
Sanity checks for the physics engine. Run: python test.py
"""
from physics import (
    Circle, AABB,
    circle_aabb_overlap,
    step_discrete, step_ccd, step_multirate,
    integrate,
)


def test_discrete_tunnels():
    wall = AABB((700, 360), half=(5, 100))
    proj = Circle((80, 360), (2000, 0), 4, restitution=0.9)
    dt = 1 / 60
    for _ in range(30):
        integrate(proj, dt)
        hit = circle_aabb_overlap(proj, wall)[0]
        if hit:
            raise AssertionError("Expected tunneling, but a discrete hit was detected")
        if proj.pos[0] > 760:
            break
    print("PASS: discrete 60 Hz tunnels through the wall")


def test_ccd_catches():
    wall = AABB((700, 360), half=(5, 100))
    proj = Circle((80, 360), (2000, 0), 4, restitution=0.9)
    dt = 1 / 60
    # ~19 frames: position ~680..713 — CCD should catch within that window
    for _ in range(20):
        step_ccd([proj], [wall], dt)
    assert proj.vel[0] < 0, f"CCD did not reflect the projectile: vx={proj.vel[0]}"
    print(f"PASS: CCD catches and reflects (vx={proj.vel[0]:.1f})")


def test_multirate_catches():
    wall = AABB((700, 360), half=(5, 100))
    proj = Circle((80, 360), (2000, 0), 4, restitution=0.9)
    dt = 1 / 60
    for _ in range(25):
        step_multirate([], [proj], [wall], dt, ratio=4)
    assert proj.vel[0] < 0, f"Multi-rate did not reflect: vx={proj.vel[0]}"
    print(f"PASS: multi-rate 240 Hz catches and reflects (vx={proj.vel[0]:.1f})")


if __name__ == "__main__":
    test_discrete_tunnels()
    test_ccd_catches()
    test_multirate_catches()
    print("\nAll tests passed.")