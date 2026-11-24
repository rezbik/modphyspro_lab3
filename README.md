# CCD vs Multi-rate — 2D physics demo

Simplified 2D rigid-body physics demo (circles + static AABBs) that compares:

1. **Discrete** collision detection at 60 Hz — fast projectile tunnels through a thin wall.
2. **CCD** — swept circle vs AABB (slab method), finds the exact contact time inside the step.
3. **Multi-rate** — ordinary objects at 60 Hz, fast projectiles at 240 Hz (4 sub-steps per frame).
4. **All-at-240 Hz** — reference: same sub-step budget, but applied to everything.

## Requirements

- Python 3.8+
- `pygame>=2.0`

## Build / install

```bash
git clone <your-repo-url>
cd ccd-multirate-demo
python -m venv .venv
source .venv/bin/activate          # Windows: .venv\Scripts\activate
pip install -r requirements.txt
```

## Run

```bash
python main.py
```

## Controls

| Key | Action |
|-----|--------|
| `1` | DISCRETE (60 Hz, naive overlap check) |
| `2` | CCD (60 Hz, swept circle vs AABB) |
| `3` | MULTIRATE (slow @ 60 Hz, fast @ 240 Hz) |
| `4` | ALL_240 (everything @ 240 Hz) |
| `R` | reset the scene |
| `ESC` | quit |

## What you should see

- **Mode 1 (DISCRETE):** the red projectile passes through the grey 10 px wall.
- **Mode 2 (CCD):** the projectile bounces off the wall — the swept test finds the contact time.
- **Mode 3 (MULTIRATE):** the projectile bounces — 240 Hz gives 4× smaller sub-steps.
- **Mode 4 (ALL_240):** same result as 3, but the physics cost per frame is higher because slow objects are also updated at 240 Hz.

The HUD prints physics time (ms/frame) and substeps per frame.

## Tests

```bash
python test.py
```

Verifies:
- discrete 60 Hz tunnels through the wall;
- CCD catches the contact;
- multi-rate at 240 Hz catches the contact.

## Implementation notes

### CCD

For a swept circle vs a static AABB, expand the AABB by the circle radius
(`min_x = bx - hx - r`, etc.) and intersect the ray
`p(t) = p0 + v * t` with each axis-aligned slab. The `t_near` of the two
slabs is the first contact time (as a fraction of the step). The normal comes
from whichever slab produced `t_near`.

`step_ccd` iterates at most 8 times per body: find earliest collision → move
there → reflect → continue with the remaining time. A tiny positional nudge
prevents re-detecting the same contact at `t=0`.

### Multi-rate

`step_multirate` does `ratio` sub-steps of `dt_frame/ratio` for fast bodies,
then one full step for slow bodies. Collision response inside each sub-step is
the plain discrete overlap + push-out + reflection.

### Determinism

`random.seed(42)` in `Scene._make_dynamics` → identical scene on every reset.
Physics steps are deterministic (no `random` in the loop).

## Known limitations

- Dynamic-vs-dynamic collisions are **not** implemented; only dynamic-vs-static.
  This keeps the demo focused on CCD / multi-rate.
- Multi-rate reduces tunneling but does not eliminate it: at 240 Hz a fast
  enough projectile can still skip a thin enough wall.
- CCD handles at most 8 contacts per body per step.
- The discrete overlap resolver assumes the body approached from a particular
  side; if a body tunnels into the *middle* of a thick wall, push-out may pick
  the wrong side.

## AI assistance

- The **math** (slab method, impulse reflection, penetration push-out) is from
  the assignment's theoretical notes and standard references.
- The **project structure, docstrings, HUD, and README** were drafted with AI
  assistance and reviewed/edited by me.
- All physics logic was tested with `test.py` and by running each mode in the
  app.