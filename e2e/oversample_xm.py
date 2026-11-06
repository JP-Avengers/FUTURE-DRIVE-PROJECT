"""train 시나리오 비중 올리기 — 원본 데이터 폴더를 복사하고 train/<시나리오>/ 의 .pqt 를 N배로 (E)

  왜    xm_ft v0.1 은 자동 교사 28,340행 · Nav2 9,871행(74:26)으로 배워 Nav2 장애물 구간에서 필요한 회전의 절반만 꺾음
        (test 급회전 |ω 교사|>0.3: 예측 0.26 · 교사 0.50, 부호 반대 20 %). xm_v1.2 = v1.1 + Nav2 train ×3 (≈ 50:50).
  방법  로더(X-Mobility)는 <split>/<시나리오>/ 안의 .pqt 를 모두 읽으므로 같은 파일을 이름만 바꿔 복사하면 그만큼 자주 나옴.
        복사본 이름 run_<묶음>_<NN + 50·k>.pqt(k = 1..N-1) — 묶음 이름이 같아 check_xm_dataset 의 split 누수 검사와 맞음.
        val · test 는 그대로(평가를 부풀리지 않음). 원본은 읽기만. index/ 는 원본 것 그대로(로더는 안 읽음).
  기록  <대상>/OVERSAMPLE.json — 원본 경로 · 배수 · 시나리오 · 만든 복사본 목록.
  확인  python e2e/check_xm_dataset.py <대상>  (train 행 수가 늘었는지)

  python e2e/oversample_xm.py C:/Users/hennr/Downloads/xm_v1_1 C:/Users/hennr/Downloads/xm_v1_2 -s r3_nav2_obs -s r3_nav2_seg -n 3
  python3 e2e/oversample_xm.py data/teacher/xm_v1.1 data/teacher/xm_v1.2 -s r3_nav2_obs -s r3_nav2_seg -n 3
"""
import argparse
import json
import os
import re
import shutil
import sys

RUN_RE = re.compile(r"^run_(.+)_(\d+)\.pqt$")
STEP = 50                                  # 복사본 번호 = 원래 NN + 50·k (export 는 묶음마다 NN 이 50 을 넘지 않음)


def main():
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(errors="replace")
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("src", help="원본 데이터 폴더(<src>/train/<시나리오>/*.pqt)")
    ap.add_argument("dst", help="새 폴더(없어야 함)")
    ap.add_argument("-s", "--scenario", action="append", required=True, help="train 에서 늘릴 시나리오 폴더(여러 번)")
    ap.add_argument("-n", "--factor", type=int, default=3, help="배수(원본 포함), 기본 3")
    a = ap.parse_args()
    if a.factor < 2:
        sys.exit("--factor 는 2 이상")
    if os.path.exists(a.dst):
        sys.exit(f"대상이 이미 있음: {a.dst} — 지우거나 다른 이름")
    for sc in a.scenario:
        if not os.path.isdir(os.path.join(a.src, "train", sc)):
            sys.exit(f"train/{sc} 없음: {a.src}")
    shutil.copytree(a.src, a.dst)
    made = []
    for sc in a.scenario:
        p = os.path.join(a.dst, "train", sc)
        for f in sorted(os.listdir(p)):
            m = RUN_RE.match(f)
            if not m:
                continue
            if int(m.group(2)) >= STEP:
                sys.exit(f"{sc}/{f}: 번호가 {STEP} 이상 — 이미 늘린 폴더를 다시 늘리려는 것 같음")
            for k in range(1, a.factor):
                g = f"run_{m.group(1)}_{int(m.group(2)) + STEP * k:02d}.pqt"
                shutil.copy2(os.path.join(p, f), os.path.join(p, g))
                made.append(f"train/{sc}/{g}")
    rec = dict(src=os.path.abspath(a.src), factor=a.factor, scenarios=a.scenario, split="train",
               naming=f"run_<묶음>_<NN+{STEP}k>.pqt = run_<묶음>_<NN>.pqt 복사", added=made)
    with open(os.path.join(a.dst, "OVERSAMPLE.json"), "w", encoding="utf-8") as fh:
        json.dump(rec, fh, ensure_ascii=False, indent=1)
    print(f"복사본 {len(made)}개 · {', '.join(a.scenario)} × {a.factor} → {a.dst}")
    print(f"확인: python e2e/check_xm_dataset.py {a.dst}")


if __name__ == "__main__":
    main()
