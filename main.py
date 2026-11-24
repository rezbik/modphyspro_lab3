"""
Interactive demo: CCD vs Multi-rate vs Discrete vs All-at-240Hz.

Controls:
    1 — DISCRETE (60 Hz, naive overlap)
    2 — CCD       (60 Hz, swept circle-vs-AABB)
    3 — MULTIRATE (slow @ 60 Hz, fast @ 240 Hz)
    4 — ALL_240   (everything @ 240 Hz)
    R — reset scene
    ESC — quit
"""
import sys

try:
    import pygame
except ImportError:
    print("pygame is not installed. Run: pip install pygame")
    sys.exit(1)

import time
import random

from physics import (
    Circle, AABB, integrate,
    step_discrete, step_ccd, step_multirate,
)

WIDTH, HEIGHT = 1200, 720
FPS = 60
DT_FRAME = 1.0 / FPS

MODES = ['DISCRETE', 'CCD', 'MULTIRATE', 'ALL_240']
PROJECTILE_SPEED = 2000.0  # px/s
WALL_THICKNESS_HALF = 5.0  # wall is 10 px thick


class Scene:
    def __init__(self):
        self.mode = 0
        self.stats = {'phys_ms': 0.0, 'substeps': 1}
        self.walls = self._make_walls()
        self.slow, self.fast = self._make_dynamics()

    # ----- construction -----

    def _make_walls(self):
        walls = []
        # thin inner wall for the CCD demo (10 px thick, 200 px tall)
        walls.append(AABB((700, 360), half=(WALL_THICKNESS_HALF, 100.0)))
        # playfield borders
        walls.append(AABB((WIDTH // 2, 5), half=(WIDTH // 2, 5)))
        walls.append(AABB((WIDTH // 2, HEIGHT - 5), half=(WIDTH // 2, 5)))
        walls.append(AABB((5, HEIGHT // 2), half=(5, HEIGHT // 2)))
        walls.append(AABB((WIDTH - 5, HEIGHT // 2), half=(5, HEIGHT // 2)))
        # three larger static obstacles (≈2× normal ball size)
        walls.append(AABB((300, 180), half=(25, 25)))
        walls.append(AABB((300, 540), half=(25, 25)))
        walls.append(AABB((950, 180), half=(25, 25)))
        return walls

    def _make_dynamics(self):
        random.seed(42)  # determinism
        slow = []
        # 15 ordinary balls, kept away from the projectile corridor (y ∈ [320, 400])
        for _ in range(15):
            x = random.randint(80, WIDTH - 80)
            if random.random() < 0.5:
                y = random.randint(80, 280)
            else:
                y = random.randint(440, HEIGHT - 80)
            r = random.randint(8, 15)
            vx = random.uniform(-200, 200)
            vy = random.uniform(-200, 200)
            color = (random.randint(100, 220),
                     random.randint(100, 220),
                     random.randint(100, 220))
            slow.append(Circle((x, y), (vx, vy), r, color=color))
        fast = [self._make_projectile()]
        return slow, fast

    def _make_projectile(self):
        p = Circle((80, 360), (PROJECTILE_SPEED, 0), 4,
                   restitution=0.9, color=(255, 80, 80))
        p.fast = True
        p.trail = []
        return p

    def reset(self):
        self.slow, self.fast = self._make_dynamics()

    # ----- simulation -----

    def step(self, dt_frame):
        mode = MODES[self.mode]
        t0 = time.perf_counter()

        if mode == 'DISCRETE':
            step_discrete(self.slow + self.fast, self.walls, dt_frame)
            self.stats['substeps'] = 1
        elif mode == 'CCD':
            step_ccd(self.slow + self.fast, self.walls, dt_frame)
            self.stats['substeps'] = 1
        elif mode == 'MULTIRATE':
            step_multirate(self.slow, self.fast, self.walls, dt_frame, ratio=4)
            self.stats['substeps'] = 4
        elif mode == 'ALL_240':
            for _ in range(4):
                step_discrete(self.slow + self.fast, self.walls, dt_frame / 4)
            self.stats['substeps'] = 4

        self.stats['phys_ms'] = (time.perf_counter() - t0) * 1000.0

        # trails + respawn logic for projectiles
        for p in list(self.fast):
            if not hasattr(p, 'trail'):
                p.trail = []
            p.trail.append((p.pos[0], p.pos[1]))
            if len(p.trail) > 30:
                p.trail.pop(0)

            speed = (p.vel[0] ** 2 + p.vel[1] ** 2) ** 0.5
            off = (p.pos[0] < -40 or p.pos[0] > WIDTH + 40 or
                   p.pos[1] < -40 or p.pos[1] > HEIGHT + 40)
            if off or speed < 600.0:
                self.fast.remove(p)
                self.fast.append(self._make_projectile())
                break

    # ----- rendering -----

    def draw(self, surf, font):
        surf.fill((22, 22, 32))

        # projectile trails first (behind everything)
        for p in self.fast:
            trail = getattr(p, 'trail', [])
            for i in range(1, len(trail)):
                a = i / len(trail)
                color = (int(255 * a), int(80 * a), int(80 * a))
                pygame.draw.line(surf, color, trail[i - 1], trail[i], 2)

        for w in self.walls:
            x, y = w.pos
            hx, hy = w.half
            pygame.draw.rect(surf, (120, 120, 130),
                             (x - hx, y - hy, hx * 2, hy * 2))

        for c in self.slow:
            pygame.draw.circle(surf, c.color,
                               (int(c.pos[0]), int(c.pos[1])),
                               int(c.radius))

        for p in self.fast:
            pygame.draw.circle(surf, p.color,
                               (int(p.pos[0]), int(p.pos[1])),
                               int(p.radius))

        # HUD
        mode = MODES[self.mode]
        lines = [
            f"Mode: {mode}",
            f"Physics: {self.stats['phys_ms']:.2f} ms/frame",
            f"Substeps/frame: {self.stats['substeps']}",
            f"Projectile: {PROJECTILE_SPEED:.0f} px/s   wall: {int(WALL_THICKNESS_HALF*2)} px",
            "",
            "[1] DISCRETE   [2] CCD   [3] MULTIRATE   [4] ALL_240",
            "[R] reset scene   [ESC] quit",
        ]
        y = 8
        for line in lines:
            surf.blit(font.render(line, True, (230, 230, 230)), (10, y))
            y += 20


def main():
    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("CCD vs Multi-rate demo")
    clock = pygame.time.Clock()
    font = pygame.font.SysFont("Consolas", 16)
    scene = Scene()

    running = True
    while running:
        clock.tick(FPS)

        for ev in pygame.event.get():
            if ev.type == pygame.QUIT:
                running = False
            elif ev.type == pygame.KEYDOWN:
                if ev.key == pygame.K_ESCAPE:
                    running = False
                elif ev.key == pygame.K_1:
                    scene.mode = 0; scene.reset()
                elif ev.key == pygame.K_2:
                    scene.mode = 1; scene.reset()
                elif ev.key == pygame.K_3:
                    scene.mode = 2; scene.reset()
                elif ev.key == pygame.K_4:
                    scene.mode = 3; scene.reset()
                elif ev.key == pygame.K_r:
                    scene.reset()

        scene.step(DT_FRAME)
        scene.draw(screen, font)
        pygame.display.flip()

    pygame.quit()
    sys.exit(0)


if __name__ == "__main__":
    main()