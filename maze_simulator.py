"""
Maze-Solving Robot Simulator
Algorithms: BFS, DFS, A*, Dijkstra
Visualization: pygame
"""

import pygame
import random
import heapq
import time
from collections import deque

# ── Colours ────────────────────────────────────────────────────────────────
BG          = (15,  17,  26)
WALL        = (30,  34,  50)
PASSAGE     = (45,  52,  80)
START_C     = (50, 220, 120)
END_C       = (220, 70,  90)
VISITED_C   = (40,  80, 160)
PATH_C      = (255, 200,  50)
ROBOT_C     = (0,  200, 255)
GRID_LINE   = (25,  28,  40)
TEXT_C      = (200, 210, 240)
PANEL_BG    = (20,  22,  34)
HIGHLIGHT   = (80,  90, 140)
BTN_ACTIVE  = (60, 140, 220)
BTN_HOVER   = (50, 110, 180)
BTN_NORMAL  = (35,  42,  65)

# ── Constants ───────────────────────────────────────────────────────────────
CELL        = 20
COLS        = 35
ROWS        = 28
PANEL_W     = 230
WIN_W       = COLS * CELL + PANEL_W
WIN_H       = ROWS * CELL
FPS         = 60


# ══════════════════════════════════════════════════════════════════════════════
# Maze generator  (recursive back-tracker / DFS carving)
# ══════════════════════════════════════════════════════════════════════════════
def generate_maze(rows, cols):
    """Return a 2-D grid: 1 = wall, 0 = passage."""
    grid = [[1] * cols for _ in range(rows)]

    def carve(r, c):
        grid[r][c] = 0
        dirs = [(0, 2), (0, -2), (2, 0), (-2, 0)]
        random.shuffle(dirs)
        for dr, dc in dirs:
            nr, nc = r + dr, c + dc
            if 0 < nr < rows and 0 < nc < cols and grid[nr][nc] == 1:
                grid[r + dr // 2][c + dc // 2] = 0
                carve(nr, nc)

    carve(1, 1)
    # Guarantee start/end are open
    grid[1][1] = 0
    grid[rows - 2][cols - 2] = 0
    return grid


# ══════════════════════════════════════════════════════════════════════════════
# Path-finding algorithms  (all return visited-order list + final path list)
# ══════════════════════════════════════════════════════════════════════════════
def neighbours(grid, r, c):
    rows, cols = len(grid), len(grid[0])
    for dr, dc in [(-1,0),(1,0),(0,-1),(0,1)]:
        nr, nc = r+dr, c+dc
        if 0 <= nr < rows and 0 <= nc < cols and grid[nr][nc] == 0:
            yield nr, nc


def reconstruct(came_from, end):
    path, cur = [], end
    while cur:
        path.append(cur)
        cur = came_from.get(cur)
    return path[::-1]


def bfs(grid, start, end):
    queue = deque([start])
    came_from = {start: None}
    visited_order = []
    while queue:
        cur = queue.popleft()
        visited_order.append(cur)
        if cur == end:
            return visited_order, reconstruct(came_from, end)
        for nb in neighbours(grid, *cur):
            if nb not in came_from:
                came_from[nb] = cur
                queue.append(nb)
    return visited_order, []


def dfs(grid, start, end):
    stack = [start]
    came_from = {start: None}
    visited_order = []
    while stack:
        cur = stack.pop()
        if cur in visited_order:
            continue
        visited_order.append(cur)
        if cur == end:
            return visited_order, reconstruct(came_from, end)
        for nb in neighbours(grid, *cur):
            if nb not in came_from:
                came_from[nb] = cur
                stack.append(nb)
    return visited_order, []


def dijkstra(grid, start, end):
    dist = {start: 0}
    came_from = {start: None}
    pq = [(0, start)]
    visited_order = []
    visited_set = set()
    while pq:
        d, cur = heapq.heappop(pq)
        if cur in visited_set:
            continue
        visited_set.add(cur)
        visited_order.append(cur)
        if cur == end:
            return visited_order, reconstruct(came_from, end)
        for nb in neighbours(grid, *cur):
            nd = d + 1
            if nd < dist.get(nb, float('inf')):
                dist[nb] = nd
                came_from[nb] = cur
                heapq.heappush(pq, (nd, nb))
    return visited_order, []


def astar(grid, start, end):
    def h(a, b):
        return abs(a[0]-b[0]) + abs(a[1]-b[1])
    g = {start: 0}
    came_from = {start: None}
    pq = [(h(start, end), 0, start)]
    visited_order = []
    visited_set = set()
    while pq:
        _, cost, cur = heapq.heappop(pq)
        if cur in visited_set:
            continue
        visited_set.add(cur)
        visited_order.append(cur)
        if cur == end:
            return visited_order, reconstruct(came_from, end)
        for nb in neighbours(grid, *cur):
            ng = cost + 1
            if ng < g.get(nb, float('inf')):
                g[nb] = ng
                came_from[nb] = cur
                heapq.heappush(pq, (ng + h(nb, end), ng, nb))
    return visited_order, []


# ══════════════════════════════════════════════════════════════════════════════
# UI helpers
# ══════════════════════════════════════════════════════════════════════════════
class Button:
    def __init__(self, rect, label, active=False):
        self.rect   = pygame.Rect(rect)
        self.label  = label
        self.active = active

    def draw(self, surf, font, mouse):
        hovered = self.rect.collidepoint(mouse)
        if self.active:
            col = BTN_ACTIVE
        elif hovered:
            col = BTN_HOVER
        else:
            col = BTN_NORMAL
        pygame.draw.rect(surf, col, self.rect, border_radius=6)
        pygame.draw.rect(surf, HIGHLIGHT, self.rect, 1, border_radius=6)
        txt = font.render(self.label, True, TEXT_C)
        surf.blit(txt, txt.get_rect(center=self.rect.center))

    def clicked(self, event):
        return (event.type == pygame.MOUSEBUTTONDOWN and
                event.button == 1 and
                self.rect.collidepoint(event.pos))


def draw_text(surf, font, text, x, y, color=TEXT_C):
    surf.blit(font.render(text, True, color), (x, y))


# ══════════════════════════════════════════════════════════════════════════════
# Simulator state machine
# ══════════════════════════════════════════════════════════════════════════════
ALGORITHMS = {
    "BFS":      bfs,
    "DFS":      dfs,
    "Dijkstra": dijkstra,
    "A*":       astar,
}

SPEEDS = {"Slow": 4, "Medium": 16, "Fast": 60}


class Simulator:
    def __init__(self):
        pygame.init()
        self.screen = pygame.display.set_mode((WIN_W, WIN_H))
        pygame.display.set_caption("Maze-Solving Robot Simulator")
        self.clock  = pygame.time.Clock()
        self.font_b = pygame.font.SysFont("monospace", 14, bold=True)
        self.font   = pygame.font.SysFont("monospace", 13)
        self.font_s = pygame.font.SysFont("monospace", 11)

        self.algo_name  = "A*"
        self.speed_name = "Medium"
        self.reset()
        self._build_buttons()

    # ── maze / solve state ────────────────────────────────────────────────
    def reset(self):
        self.grid        = generate_maze(ROWS, COLS)
        self.start       = (1, 1)
        self.end         = (ROWS - 2, COLS - 2)
        self.visited_seq = []
        self.path        = []
        self.state       = "idle"      # idle | searching | done
        self.vis_idx     = 0
        self.path_idx    = 0
        self.robot_pos   = self.start
        self.stats       = {}
        self.visited_set = set()
        self.path_set    = set()

    def solve(self):
        fn  = ALGORITHMS[self.algo_name]
        t0  = time.perf_counter()
        v, p = fn(self.grid, self.start, self.end)
        dt  = time.perf_counter() - t0
        self.visited_seq = v
        self.path        = p
        self.stats = {
            "nodes":  len(v),
            "path":   len(p),
            "time_ms": f"{dt*1000:.2f}",
        }
        self.state     = "searching"
        self.vis_idx   = 0
        self.path_idx  = 0
        self.visited_set.clear()
        self.path_set.clear()
        self.robot_pos = self.start

    # ── buttons ───────────────────────────────────────────────────────────
    def _build_buttons(self):
        px = COLS * CELL + 15
        self.btn_new   = Button((px, 15,  200, 32), "New Maze")
        self.btn_solve = Button((px, 55,  200, 32), "Solve")
        self.btn_reset = Button((px, 95,  200, 32), "Reset View")

        # algorithm buttons
        self.algo_btns = {}
        for i, name in enumerate(ALGORITHMS):
            self.algo_btns[name] = Button(
                (px, 155 + i * 38, 200, 30), name,
                active=(name == self.algo_name))

        # speed buttons
        self.speed_btns = {}
        for i, name in enumerate(SPEEDS):
            self.speed_btns[name] = Button(
                (px, 330 + i * 38, 200, 30), name,
                active=(name == self.speed_name))

    def _update_active(self):
        for n, b in self.algo_btns.items():
            b.active = (n == self.algo_name)
        for n, b in self.speed_btns.items():
            b.active = (n == self.speed_name)

    # ── main loop ─────────────────────────────────────────────────────────
    def run(self):
        running = True
        while running:
            mouse = pygame.mouse.get_pos()
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                if event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                    running = False

                if self.btn_new.clicked(event):
                    self.reset()
                    self._build_buttons()

                if self.btn_solve.clicked(event):
                    if self.state == "idle":
                        self.solve()

                if self.btn_reset.clicked(event):
                    self.state = "idle"
                    self.vis_idx = self.path_idx = 0
                    self.visited_set.clear()
                    self.path_set.clear()
                    self.robot_pos = self.start
                    self.stats = {}

                for name, btn in self.algo_btns.items():
                    if btn.clicked(event):
                        self.algo_name = name
                        self._update_active()

                for name, btn in self.speed_btns.items():
                    if btn.clicked(event):
                        self.speed_name = name
                        self._update_active()

            # ── advance animation ─────────────────────────────────────────
            steps = SPEEDS[self.speed_name]
            if self.state == "searching":
                for _ in range(steps):
                    if self.vis_idx < len(self.visited_seq):
                        self.visited_set.add(self.visited_seq[self.vis_idx])
                        self.robot_pos = self.visited_seq[self.vis_idx]
                        self.vis_idx += 1
                    else:
                        self.state = "pathing"
                        break

            elif self.state == "pathing":
                for _ in range(max(1, steps // 4)):
                    if self.path_idx < len(self.path):
                        self.path_set.add(self.path[self.path_idx])
                        self.robot_pos = self.path[self.path_idx]
                        self.path_idx += 1
                    else:
                        self.state = "done"
                        break

            # ── draw ──────────────────────────────────────────────────────
            self.screen.fill(BG)
            self._draw_grid()
            self._draw_panel(mouse)
            pygame.display.flip()
            self.clock.tick(FPS)

        pygame.quit()

    # ── rendering ────────────────────────────────────────────────────────
    def _draw_grid(self):
        surf = self.screen
        for r in range(ROWS):
            for c in range(COLS):
                x, y = c * CELL, r * CELL
                cell = (r, c)

                if self.grid[r][c] == 1:
                    color = WALL
                elif cell in self.path_set:
                    color = PATH_C
                elif cell in self.visited_set:
                    color = VISITED_C
                else:
                    color = PASSAGE

                pygame.draw.rect(surf, color, (x, y, CELL-1, CELL-1))

        # start / end markers
        sr, sc = self.start
        er, ec = self.end
        pygame.draw.rect(surf, START_C, (sc*CELL, sr*CELL, CELL-1, CELL-1))
        pygame.draw.rect(surf, END_C,   (ec*CELL, er*CELL, CELL-1, CELL-1))

        # robot
        rr, rc = self.robot_pos
        cx, cy = rc*CELL + CELL//2, rr*CELL + CELL//2
        pygame.draw.circle(surf, ROBOT_C, (cx, cy), CELL//2 - 1)
        pygame.draw.circle(surf, (200, 240, 255), (cx, cy), CELL//4)

    def _draw_panel(self, mouse):
        px = COLS * CELL
        pygame.draw.rect(self.screen, PANEL_BG, (px, 0, PANEL_W, WIN_H))
        pygame.draw.line(self.screen, HIGHLIGHT, (px, 0), (px, WIN_H), 1)

        x = px + 15

        # title
        draw_text(self.screen, self.font_b, "MAZE  ROBOT", x, 140, BTN_ACTIVE)

        # buttons
        self.btn_new.draw(self.screen,   self.font_b, mouse)
        self.btn_solve.draw(self.screen, self.font_b, mouse)
        self.btn_reset.draw(self.screen, self.font_b, mouse)

        # algo section
        draw_text(self.screen, self.font_b, "Algorithm", x, 135, HIGHLIGHT)
        for btn in self.algo_btns.values():
            btn.draw(self.screen, self.font, mouse)

        # speed section
        draw_text(self.screen, self.font_b, "Speed", x, 318, HIGHLIGHT)
        for btn in self.speed_btns.values():
            btn.draw(self.screen, self.font, mouse)

        # state label
        state_labels = {
            "idle":     ("READY",     START_C),
            "searching":("SEARCHING", BTN_ACTIVE),
            "pathing":  ("TRACING",   PATH_C),
            "done":     ("COMPLETE",  END_C),
        }
        label, col = state_labels.get(self.state, ("?", TEXT_C))
        draw_text(self.screen, self.font_b, f"Status: {label}", x, 460, col)

        # stats
        if self.stats:
            draw_text(self.screen, self.font_b, "Stats", x, 490, HIGHLIGHT)
            draw_text(self.screen, self.font_s,
                      f"Nodes visited : {self.stats['nodes']}", x, 510, TEXT_C)
            draw_text(self.screen, self.font_s,
                      f"Path length   : {self.stats['path']}", x, 526, TEXT_C)
            draw_text(self.screen, self.font_s,
                      f"Solve time    : {self.stats['time_ms']} ms", x, 542, TEXT_C)

        # legend
        legend_y = 580
        draw_text(self.screen, self.font_b, "Legend", x, legend_y, HIGHLIGHT)
        legend = [
            (START_C,   "Start"),
            (END_C,     "End"),
            (VISITED_C, "Explored"),
            (PATH_C,    "Optimal path"),
            (ROBOT_C,   "Robot"),
            (WALL,      "Wall"),
        ]
        for i, (col, lbl) in enumerate(legend):
            ly = legend_y + 20 + i * 18
            pygame.draw.rect(self.screen, col, (x, ly+2, 12, 12), border_radius=2)
            draw_text(self.screen, self.font_s, lbl, x+18, ly, TEXT_C)

        # footer hint
        draw_text(self.screen, self.font_s,
                  "ESC to quit", x, WIN_H - 20, HIGHLIGHT)


# ══════════════════════════════════════════════════════════════════════════════
if __name__ == "__main__":
    Simulator().run()
