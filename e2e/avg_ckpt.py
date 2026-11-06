"""체크포인트 가중치 평균(model soup) — 같은 사전학습 ckpt 에서 미세조정한 Lightning ckpt 여러 개의 state_dict 를 평균해 하나로 (E)

  왜    Model soups(Wortsman 2022): 같은 초기화 · 작은 lr 로 미세조정한 가중치들은 같은 손실 골짜기에 있어 평균이 단일 best 보다
        좋은 일이 많고 추론 비용은 그대로. 우리 ckpt 는 모두 nvidia/X-Mobility 에서 lr 1e-5 → 후보: v0.1 + v0.2 (+ aw30), 에폭 7–9.
  방법  첫 ckpt 를 통째로 복사하고 state_dict 의 실수 텐서만 산술 평균(가중치 --weights). 정수 버퍼(num_batches_tracked 등)는
        첫 ckpt 값. 키 집합이 다르면 멈춤(--allow-missing 이면 공통 키만 평균, 나머지는 첫 ckpt).
  평가  python e2e/eval_offline.py ... -p <평균.ckpt> -o <폴더>  (학습 없이 평가만)
  주의  torch 필요(Kaggle · 서버 · 집 PC). 이 PC(torch 없음)에서는 못 돌림 — 11.7 작성 시 미실행.

  python e2e/avg_ckpt.py -o soup_v01_v02.ckpt a/checkpoints/epoch=8-step=10719.ckpt b/checkpoints/epoch=9-step=20000.ckpt
"""
import argparse
import hashlib
import json
import os
import sys


def main():
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(errors="replace")
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("ckpts", nargs="+", help="Lightning .ckpt 2개 이상")
    ap.add_argument("-o", "--out", required=True)
    ap.add_argument("--weights", help="쉼표로 나눈 가중치(기본 균등), 예: 0.5,0.5")
    ap.add_argument("--allow-missing", action="store_true", help="키 집합이 달라도 공통 키만 평균")
    a = ap.parse_args()
    import torch
    if len(a.ckpts) < 2:
        sys.exit("ckpt 2개 이상")
    w = [float(x) for x in a.weights.split(",")] if a.weights else [1.0 / len(a.ckpts)] * len(a.ckpts)
    if len(w) != len(a.ckpts) or abs(sum(w) - 1.0) > 1e-6:
        sys.exit("--weights 개수가 ckpt 수와 같고 합이 1 이어야 함")
    if os.path.exists(a.out):
        sys.exit(f"이미 있음: {a.out}")
    base = torch.load(a.ckpts[0], map_location="cpu", weights_only=False)
    sd0 = base["state_dict"]
    keys = set(sd0)
    others = []
    for p in a.ckpts[1:]:
        sd = torch.load(p, map_location="cpu", weights_only=False)["state_dict"]
        if set(sd) != keys:
            diff = sorted(keys ^ set(sd))
            if not a.allow_missing:
                sys.exit(f"키 집합이 다름({len(diff)}개, 예 {diff[:3]}) — --allow-missing 이면 공통 키만 평균")
            keys &= set(sd)
        others.append(sd)
    n_avg, n_keep = 0, 0
    for k in sorted(sd0):
        t = sd0[k]
        if k in keys and torch.is_tensor(t) and t.is_floating_point() and all(k in sd and sd[k].shape == t.shape for sd in others):
            acc = t.to(torch.float32) * w[0]
            for wi, sd in zip(w[1:], others):
                acc = acc + sd[k].to(torch.float32) * wi
            sd0[k] = acc.to(t.dtype)
            n_avg += 1
        else:
            n_keep += 1
    base["state_dict"] = sd0
    base.setdefault("hyper_parameters", {})
    base["soup"] = dict(sources=[os.path.abspath(p) for p in a.ckpts], weights=w, averaged=n_avg, kept_from_first=n_keep)
    torch.save(base, a.out)
    h = hashlib.sha256()
    with open(a.out, "rb") as fh:
        for chunk in iter(lambda: fh.read(1 << 24), b""):
            h.update(chunk)
    print(json.dumps(dict(out=a.out, averaged=n_avg, kept_from_first=n_keep, weights=w,
                          size_gb=round(os.path.getsize(a.out) / 2**30, 2), sha12=h.hexdigest()[:12]), ensure_ascii=False))


if __name__ == "__main__":
    main()
