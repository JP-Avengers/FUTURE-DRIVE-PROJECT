"""scan2sim — 실내 스캔을 넣으면 Isaac Sim 에 올릴 맵 묶음과 진단 보고서를 만든다.

    cd recon
    python -m scan2sim <입력> --out <출력 폴더> [--force]

입력: GLB 파일(스캔 앱 결과) 또는 B 형식 맵 폴더(keyframes/ + trajectory_camera_optical.txt [+ keyframes_<이름>/ · geometry*.ply]).
출력: <out>/bundle/ (usd · nav2 · scene_root.usda · start_pose.json · route.csv) · report.md · stages/*.json · log.txt
종료 코드: 0 = 통과 또는 주의 · 1 = 실패 (어느 단계에서 왜인지는 report.md)
"""
import argparse
import os
import sys

from .core import FAIL, Context, run
from .stages import STAGES


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("input")
    ap.add_argument("--out", required=True)
    ap.add_argument("--force", action="store_true", help="실패한 단계가 있어도 다음 단계를 해 봄")
    ap.add_argument("--resume", action="store_true", help="<out>/build 에 이미 만든 결과가 있으면 변환을 건너뛰고 진단 · 묶음만 다시")
    a = ap.parse_args()
    ctx = Context(input=os.path.abspath(os.path.expanduser(a.input)), out=os.path.abspath(os.path.expanduser(a.out)), resume=a.resume)
    os.makedirs(ctx["out"], exist_ok=True)
    res = run(STAGES, ctx, force=a.force)
    print(f"\n보고서: {os.path.join(ctx['out'], 'report.md')}")
    sys.exit(1 if any(d["status"] == FAIL for d in res) else 0)


if __name__ == "__main__":
    main()
