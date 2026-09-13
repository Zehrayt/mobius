"""
Step 1 proof-of-concept: "organik/fiziksel" (Verlet integration chain) vs
"robotik/dikte" (pure sinusoidal kinematics) karşılaştırması.

Aynı anchor (gövde/kafa) hareketini iki tarafa da veriyoruz:
  SOL panel : anchor'a iple bağlı bir "kuyruk/kol" zinciri, Verlet integration +
              distance constraint ile çözülüyor (yerçekimi + sürtünme var,
              hareketin momentumunu/atalet etkisini hissediyor).
  SAĞ panel : aynı uzunlukta zincir ama her eklem saf sin() formülüyle
              döndürülüyor (Gemini konuşmasındaki "bacak açısı = sin(zaman)x30"
              yöntemi) -- anchor'ın ani yön değiştirmelerine hiç tepki vermiyor.

Anchor kasıtlı olarak ani yön değiştiriyor (üçgen dalga) ki atalet/savrulma
farkı gözle net görülsün.
"""
import os
import numpy as np
import cv2

W, H = 480, 360
FPS = 30
DURATION_S = 6
N_FRAMES = FPS * DURATION_S
N_POINTS = 8          # zincirdeki nokta sayısı (anchor dahil)
SEG_LEN = 26          # her segmentin dinlenme uzunluğu (px)
GRAVITY = np.array([0.0, 900.0])   # px/s^2
DAMPING = 0.985
RELAX_ITERS = 10

def anchor_pos(t):
    """Ani yön değiştiren üçgen dalga yatay hareket + sabit yükseklik."""
    period = 2.0
    phase = (t % period) / period
    tri = 4 * abs(phase - 0.5) - 1   # -1..1 üçgen dalga
    x = W * 0.5 + tri * (W * 0.32)
    y = H * 0.32
    return np.array([x, y])

class VerletChain:
    def __init__(self, start):
        self.pos = np.array([start + np.array([0, i * SEG_LEN]) for i in range(N_POINTS)], dtype=float)
        self.prev = self.pos.copy()

    def step(self, dt, anchor):
        # Verlet integration
        vel = (self.pos - self.prev) * DAMPING
        new_pos = self.pos + vel + GRAVITY * dt * dt
        self.prev = self.pos.copy()
        self.pos = new_pos
        # anchor (index 0) pinned to driver
        self.pos[0] = anchor
        # distance constraint relaxation
        for _ in range(RELAX_ITERS):
            for i in range(N_POINTS - 1):
                p1, p2 = self.pos[i], self.pos[i + 1]
                delta = p2 - p1
                dist = np.linalg.norm(delta) + 1e-9
                diff = (dist - SEG_LEN) / dist
                if i == 0:
                    # ilk nokta pinned, tüm düzeltme ikinci noktaya gitsin
                    self.pos[i + 1] -= delta * diff
                else:
                    self.pos[i] += delta * diff * 0.5
                    self.pos[i + 1] -= delta * diff * 0.5
            self.pos[0] = anchor  # her iterasyonda anchor'ı yeniden sabitle

class RoboticChain:
    """Fiziksiz, saf sinüs formülüyle her eklemi döndüren 'dikte' zincir."""
    def __init__(self):
        self.t0 = 0.0

    def points(self, t, anchor):
        pts = [anchor.copy()]
        for i in range(1, N_POINTS):
            angle = np.sin(t * 3.0 + i * 0.9) * 0.6 + (np.pi / 2)
            prev = pts[-1]
            nxt = prev + SEG_LEN * np.array([np.cos(angle), np.sin(angle)])
            pts.append(nxt)
        return np.array(pts)

def draw_chain(frame, pts, offset_x, color):
    for i in range(len(pts) - 1):
        p1 = (int(pts[i][0] + offset_x), int(pts[i][1]))
        p2 = (int(pts[i + 1][0] + offset_x), int(pts[i + 1][1]))
        cv2.line(frame, p1, p2, color, 3, cv2.LINE_AA)
    for p in pts:
        c = (int(p[0] + offset_x), int(p[1]))
        cv2.circle(frame, c, 5, color, -1, cv2.LINE_AA)
    # anchor'ı "kafa" gibi vurgula
    head = (int(pts[0][0] + offset_x), int(pts[0][1]))
    cv2.circle(frame, head, 10, color, 2, cv2.LINE_AA)

def main():
    verlet = VerletChain(anchor_pos(0))
    robotic = RoboticChain()
    out = cv2.VideoWriter(
        os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "outputs", "step1_verlet_vs_robotic.mp4"),
        cv2.VideoWriter_fourcc(*"mp4v"), FPS, (W * 2, H)
    )
    dt = 1.0 / FPS
    for f in range(N_FRAMES):
        t = f * dt
        a = anchor_pos(t)
        verlet.step(dt, a)
        rob_pts = robotic.points(t, a)

        frame = np.full((H, W * 2, 3), 24, dtype=np.uint8)
        cv2.line(frame, (W, 0), (W, H), (60, 60, 60), 2)
        cv2.putText(frame, "VERLET (organik / fizik)", (14, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (90, 220, 90), 2, cv2.LINE_AA)
        cv2.putText(frame, "SIN() KINEMATIK (robotik)", (W + 14, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (90, 150, 240), 2, cv2.LINE_AA)

        draw_chain(frame, verlet.pos, 0, (90, 220, 90))
        draw_chain(frame, rob_pts, W, (90, 150, 240))

        out.write(frame)
    out.release()
    print("done")

if __name__ == "__main__":
    main()
