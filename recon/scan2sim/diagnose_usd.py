"""이미 만든 맵을 진단만 한다 (변환 · 보정 없이) — 다른 방법으로 만든 트윈이나 예전 판본을 같은 기준으로 재 볼 때.

    cd recon
    python -m scan2sim.diagnose_usd --usd <mesh.usdc> --nav2 <map.pgm · map.yaml 폴더> [--build <ground.npz · visual.ply 폴더>] [--ceiling] [--out <결과 json>]

--nav2 가 없으면 sim/nav2/make_map.py 로 임시 지도를 만든다. --build 가 없으면 바닥 높이 0 으로 보고, 보이는 바닥 대조는 건너뛴다.
종료 코드: 0 = 통과 또는 주의 · 1 = 실패
"""
import argparse
import json
import os
import subprocess
import sys
import tempfile

from .checks import load_nav2
from .core import FAIL, MARK, Context
from .stages import REPO, s_diagnose


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--usd", required=True)
    ap.add_argument("--nav2")
    ap.add_argument("--build")
    ap.add_argument("--ceiling", action="store_true", help="천장이 있는 맵")
    ap.add_argument("--out")
    a = ap.parse_args()
    nav = a.nav2
    if not nav:
        nav = tempfile.mkdtemp(prefix="s2s_nav2_")
        subprocess.run([sys.executable, os.path.join(REPO, "sim", "nav2", "make_map.py"), a.usd, nav, "--route", ""], check=True, stdout=subprocess.DEVNULL)
    info, free, occ = load_nav2(nav)
    ctx = Context(out=tempfile.mkdtemp(prefix="s2s_diag_"), usd_mesh=os.path.abspath(a.usd), build=os.path.abspath(a.build) if a.build else "", nav2=info, nav2_free=free, nav2_occ=occ,
                  profile=dict(has_ceiling=a.ceiling), diagnose_only=True)
    r = s_diagnose(ctx)
    d = dict(usd=os.path.basename(os.path.dirname(os.path.dirname(os.path.abspath(a.usd)))) + "/" + os.path.basename(a.usd), status=r.status, summary=r.summary, metrics=r.metrics, messages=r.messages)
    print(f"[{MARK[r.status]}] {r.summary}")
    for m in r.messages:
        print("   -", m)
    if a.out:
        json.dump(d, open(a.out, "w", encoding="utf-8"), ensure_ascii=False, indent=1, default=str)
    sys.exit(1 if r.status == FAIL else 0)


if __name__ == "__main__":
    main()
