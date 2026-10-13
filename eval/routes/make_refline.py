#!/usr/bin/env python3
"""기준선 생성 — 화단(섬) 윤곽 꼭짓점을 d m 바깥으로 둥글게 부풀린 닫힌 선. 시스템 python3(PyYAML)로:

    python3 eval/routes/make_refline.py eval/routes/parking_r3_island.yaml

입력 <이름>_island.yaml — island(시계 방향 꼭짓점 [이름, x, y]) · d · step · start_after(이 꼭짓점 모서리 원호가 끝나는 곳
= 다음 변의 시작에서 출발). 변은 d 만큼 평행 이동하고 꼭짓점은 반경 d 원호로 잇는다(볼록 다각형만 — 오목하면 거절).
출력(입력 파일 옆, 고치지 말고 다시 생성):
  <이름>_cw.csv · <이름>_ccw.csv — step 간격 점 run,s,x,y (닫는 점 포함). sim/teacher.py --route csv:<파일> 로 바로 쓴다
  <이름>_cw.yaml · <이름>_ccw.yaml — 길이 · 출발 자세(씬 시작 자세와 같게 둔다) · 회전 · 경로 지시 구간(sim/teacher_core.py 와 같은 규칙)
장애물 여유 검사 · 위에서 본 그림은 메쉬가 필요해 서버 logs/refline/ 도구로 한다(볼트 '기준선 초안' 노트).
"""
import hashlib
import math
import os
import sys

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "..", "sim"))
import teacher_core as tc  # noqa: E402


def offset_ring(verts, d, arc_step=0.05):
    """시계 방향 볼록 다각형 verts(첫 꼭짓점 = start_after)를 d 만큼 부풀린 닫힌 점열.
    첫 점 = 첫 꼭짓점의 모서리 원호가 끝나는 곳(= 첫 변 시작). 변은 직선, 꼭짓점은 반경 d 원호."""
    n = len(verts)
    tans, nors = [], []
    for i in range(n):
        (ax, ay), (bx, by) = verts[i], verts[(i + 1) % n]
        L = math.hypot(bx - ax, by - ay)
        t = ((bx - ax) / L, (by - ay) / L)
        tans.append(t)
        nors.append((-t[1], t[0]))  # 시계 방향 다각형의 바깥 = 진행 방향 왼쪽
    pts = []
    for i in range(n):
        (ax, ay), nrm = verts[i], nors[i]
        pts.append((ax + d * nrm[0], ay + d * nrm[1]))                      # 변 i 시작
        bx, by = verts[(i + 1) % n]
        pts.append((bx + d * nrm[0], by + d * nrm[1]))                      # 변 i 끝
        n1 = nors[(i + 1) % n]
        a0, a1 = math.atan2(nrm[1], nrm[0]), math.atan2(n1[1], n1[0])
        sweep = (a1 - a0 + math.pi) % (2 * math.pi) - math.pi
        k = max(2, int(abs(sweep) * d / arc_step))
        for j in range(1, k):                                                # 꼭짓점 i+1 의 원호(양 끝 제외)
            a = a0 + sweep * j / k
            pts.append((bx + d * math.cos(a), by + d * math.sin(a)))
    return pts, tans


def resample(pts, step):
    ring = pts + [pts[0]]
    cum = [0.0]
    for (ax, ay), (bx, by) in zip(ring, ring[1:]):
        cum.append(cum[-1] + math.hypot(bx - ax, by - ay))
    L = cum[-1]
    n = int(round(L / step))
    out, j = [], 0
    for k in range(n):
        s = L * k / n
        while cum[j + 1] < s:
            j += 1
        seg = cum[j + 1] - cum[j]
        t = 0.0 if seg < 1e-12 else (s - cum[j]) / seg
        out.append((ring[j][0] + t * (ring[j + 1][0] - ring[j][0]), ring[j][1] + t * (ring[j + 1][1] - ring[j][1])))
    return out, L


def main():
    src = sys.argv[1] if len(sys.argv) > 1 else os.path.join(HERE, "parking_r3_island.yaml")
    raw = open(src, "rb").read()
    cfg = yaml.safe_load(raw)
    name, d, step = cfg["name"], float(cfg["d"]), float(cfg.get("step", 0.25))
    isl = [(str(v[0]), float(v[1]), float(v[2])) for v in cfg["island"]]
    names = [v[0] for v in isl]
    area = 0.5 * sum(isl[i][1] * isl[(i + 1) % len(isl)][2] - isl[(i + 1) % len(isl)][1] * isl[i][2] for i in range(len(isl)))
    if area > 0:
        raise SystemExit("island 은 시계 방향으로 적어 주세요(지금은 반시계)")
    for i in range(len(isl)):
        (_, ax, ay), (_, bx, by), (_, cx, cy) = isl[i - 1], isl[i], isl[(i + 1) % len(isl)]
        if (bx - ax) * (cy - by) - (by - ay) * (cx - bx) > 0:
            raise SystemExit(f"꼭짓점 {isl[i][0]} 에서 오목함 — 볼록 다각형만 지원")
    k0 = names.index(cfg.get("start_after", names[0]))
    verts = [(x, y) for _, x, y in isl[k0:] + isl[:k0]]
    ring, tans = offset_ring(verts, d)
    cw, L = resample(ring, step)
    ccw = [cw[0]] + cw[1:][::-1]
    yaw_cw = math.degrees(math.atan2(tans[0][1], tans[0][0]))
    sha = hashlib.sha256(raw).hexdigest()[:12]
    base = os.path.join(os.path.dirname(os.path.abspath(src)), name)
    for tag, pts, yaw in (("cw", cw, yaw_cw), ("ccw", ccw, (yaw_cw + 360) % 360 - 180)):
        with open(f"{base}_{tag}.csv", "w") as f:
            f.write("run,s,x,y\n")
            for i, (x, y) in enumerate(pts + [pts[0]]):
                f.write(f"0,{i * L / len(pts):.3f},{x:.3f},{y:.3f}\n")
        r = tc.Route(pts, step=0)
        wins = tc.turn_windows(r)
        cmds = []
        for a, b, c, deg, apex in sorted(wins, key=lambda w: w[4]):
            cmds.append(f"  - {{from_s: {a:.1f}, to_s: {b:.1f}, cmd: {'left' if c == tc.LEFT else 'right'}, "
                        f"turn_deg: {deg:.0f}, apex_s: {apex:.1f}}}")
        with open(f"{base}_{tag}.yaml", "w") as f:
            f.write(f"# 생성 파일 — 고치지 말고 {os.path.basename(src)} 를 고친 뒤 eval/routes/make_refline.py 로 다시 만든다\n")
            f.write(f"name: {name}_{tag}\nversion: {cfg.get('version', 'v0-draft')}\nscene: {cfg.get('scene', '')}\n")
            f.write(f"direction: {tag}\nsource: {os.path.basename(src)} (sha256 {sha})\n")
            f.write(f"rule: 화단 윤곽을 {d:g} m 바깥으로 부풀린 선 — 변은 평행 이동, 꼭짓점은 반경 {d:g} m 원호\n")
            f.write(f"ref_path_csv: {os.path.basename(base)}_{tag}.csv   # {step:g} m 간격 run,s,x,y (닫는 점 포함)\n")
            f.write(f"length_m: {L:.2f}\n")
            f.write(f"start: {{x: {pts[0][0]:.3f}, y: {pts[0][1]:.3f}, yaw_deg: {yaw:.1f}, s: 0.0}}   "
                    f"# {cfg.get('start_after', names[0])} 모서리 원호 끝 — 씬 시작 자세와 같게\n")
            f.write("commands:   # /route_command — 40° 넘는 꺾임 꼭짓점 앞 5 m ~ 뒤 2 m(sim/teacher_core.py turn_windows), 그 밖은 straight. from_s > to_s 면 바퀴 이음을 넘는 구간\n")
            f.write("\n".join(cmds) + "\n")
        print(f"{tag}: L {L:.2f} m · 점 {len(pts)} · 출발 ({pts[0][0]:.3f}, {pts[0][1]:.3f}) yaw {yaw:.1f}° · "
              f"회전 {len(wins)} — {base}_{tag}.csv · .yaml")


if __name__ == "__main__":
    main()
