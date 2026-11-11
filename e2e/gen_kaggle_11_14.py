"""e2e/gen_kaggle_11_14.py — 11.14(토) 09:00 Kaggle 할당량 갱신 때 띄울 학습 push 폴더 생성기 — 레포의 *현재* e2e/kaggle_finetune.ipynb 에서 만든다.

쓰는 법(Git Bash, 레포에서):  PYTHONIOENCODING=utf-8 python e2e/gen_kaggle_11_14.py [--only 이름 ...] [--v15-dataset henryseo711/dataset-08-xm-v1-5]
출력: ~/Downloads/kaggle_runs_11_14/<이름>/<커널>.ipynb + kernel-metadata.json → 그 폴더에서 `python -m kaggle kernels push`.
레포 파일은 건드리지 않음. 노트북이 smoke 실측 스텝 시간으로 에폭 수를 다시 정하므로 MAX_EPOCHS 는 상한.
"""
import argparse, copy, glob, json, math, os, re

REPO = os.path.expanduser("~/FUTURE-DRIVE-PROJECT")
NB_FT = f"{REPO}/e2e/kaggle_finetune.ipynb"
HERE = os.path.expanduser("~/Downloads/kaggle_runs_11_14")   # push 폴더 자리(레포 밖)
USER = "henryseo711"
PRE = f"{USER}/fd-xm-pretrained"            # 사전학습 ckpt → HF 토큰 없이 CLI push 로 돎
V14 = f"{USER}/dataset-07-xm-v1-4"
IMAGE = "gcr.io/kaggle-private-byod/python@sha256:de153e6d303881930c51286216393d56f56b26458d0661350305a1e8144eea01"
KERNELS = ("fd-xm-finetune", "fd-xm-finetune-aw30")
AW30 = "--gin XMobilityLoss.action_weight=30"
EXEC = "--prev-action-source exec"
PAD05 = "--prev-action-dropout 0.5"

# 행 수(train): v1.4 = 68,441(RESPLIT · 11.10 확인), xm_v08r3_v1 train = 28,548 · xm_v08r3_nav2_v1 train = 15,551(MANIFEST 11.11) → v1.5 = 112,540.
# 시퀀스 ≈ 행 / 4(xm_v1 38k행 → 9,529 · v1.3 48,082 → 11,987 실측 비 0.249)
ROWS = {"v1.4": 68441, "v1.5": 68441 + 28548 + 15551}
STEP_S = 0.66                                  # Kaggle T4×2 본 학습 실측(v0.1 0.655 · aw30 0.657)
WORLD, BATCH, ACCUM = 2, 2, 2


def plan(rows, budget_h, max_epochs):
    seq = int(rows * 0.249)
    spe = math.ceil(seq / WORLD) // BATCH
    per = spe * STEP_S * 1.15                  # 노트북 칸 8 과 같은 식
    fit = int((budget_h * 3600 - 1200) // per)
    ep = min(max_epochs, fit)
    return dict(seq=seq, spe=spe, per_min=per / 60, fit=fit, epochs=ep, hours=per * ep / 3600, opt_steps=ep * math.ceil(spe / ACCUM))


def runs(v15_ds):
    return {
        # v1.4 — 데이터 이미 있음. 집 PC E4(xm_ft_v1.4_aw30_exec_e10) 결과 보고 고름
        "v14_exec_pad05": dict(run="xm_ft_v1.4_aw30_exec_pad05", extra=f"{AW30} {EXEC} {PAD05}", data=[V14, PRE], ver="v1.4",
                               max_epochs=10, budget=10.0, kernel="fd-xm-finetune",
                               note="E5 = E4 + 직전 명령 드롭아웃 0.5. E1(드롭아웃) 채택 시 1순위"),
        "v14_exec_seed1": dict(run="xm_ft_v1.4_aw30_exec_seed1", extra=f"{AW30} {EXEC} --seed 1", data=[V14, PRE], ver="v1.4",
                               max_epochs=10, budget=10.0, kernel="fd-xm-finetune-aw30",
                               note="E4 시드 반복(규칙 3) — E4 가 aw30 보다 나으면 '시드 차 안인지' 확인용"),
        "v14_exec": dict(run="xm_ft_v1.4_aw30_exec", extra=f"{AW30} {EXEC}", data=[V14, PRE], ver="v1.4",
                         max_epochs=10, budget=10.0, kernel="fd-xm-finetune",
                         note="E4 와 같은 설정 Kaggle 판 — 집 PC E4 가 죽었을 때만"),
        # E4 가 판정에서 떨어졌을 때(실행 명령 입력이 효과 없음) — 입력은 교사 명령 그대로
        "v14_pad05": dict(run="xm_ft_v1.4_aw30_pad05", extra=f"{AW30} {PAD05}", data=[V14, PRE], ver="v1.4",
                          max_epochs=10, budget=10.0, kernel="fd-xm-finetune",
                          note="E4 불통과 · E1 채택일 때: v1.4(잡음 · DAgger 포함) + 드롭아웃만"),
        "v15_aw30": dict(run="xm_ft_v1.5_aw30", extra=AW30, data=[v15_ds, PRE], ver="v1.5",
                         max_epochs=6, budget=10.5, kernel="fd-xm-finetune-aw30",
                         note="E4 · E1 둘 다 불통과일 때: 새 맵 데이터만 더한 aw30"),
        # v1.5 — D 의 xm_v08r3_v1 이 와서 데이터셋을 만든 뒤. 10 에폭은 12 h 에 안 들어가 6 에폭(에폭 ≈ 89분, = v1.4 10 에폭과 최적화 스텝 거의 같음)
        "v15_exec": dict(run="xm_ft_v1.5_aw30_exec", extra=f"{AW30} {EXEC}", data=[v15_ds, PRE], ver="v1.5",
                         max_epochs=6, budget=10.5, kernel="fd-xm-finetune",
                         note="v1.5(= v1.4 + v0.8 r3 자동 교사 45바퀴) × E4 설정"),
        "v15_exec_pad05": dict(run="xm_ft_v1.5_aw30_exec_pad05", extra=f"{AW30} {EXEC} {PAD05}", data=[v15_ds, PRE], ver="v1.5",
                               max_epochs=6, budget=10.5, kernel="fd-xm-finetune-aw30",
                               note="v1.5 × E5 설정"),
    }


def src(c):
    return "".join(c["source"])


def setp(s, name, value):
    pat = re.compile(rf"^({re.escape(name)} = )([^#\n]*?)(\s*#.*)?$", re.M)
    found = pat.findall(s)
    assert len(found) == 1, f"{name} 줄이 {len(found)}개"
    old = found[0][1]
    return pat.sub(lambda m: f"{m.group(1)}{value.ljust(len(old))}{m.group(3) or ''}", s, count=1)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--only", action="append", default=[])
    ap.add_argument("--v15-dataset", default=f"{USER}/dataset-08-xm-v1-5", help="v1.5 Kaggle 데이터셋(만든 뒤 이름 맞출 것)")
    ap.add_argument("--out", default=HERE)
    a = ap.parse_args()
    R = runs(a.v15_dataset)
    want = set(a.only) or set(R)
    assert want <= set(R), f"모르는 이름: {want - set(R)}"
    base = json.load(open(NB_FT, encoding="utf-8"))
    for name in R:
        if name not in want:
            continue
        cfg = R[name]
        p = plan(ROWS[cfg["ver"]], cfg["budget"], cfg["max_epochs"])
        assert p["epochs"] == cfg["max_epochs"], f"{name}: 예산 {cfg['budget']} h 에 {p['fit']} 에폭만 들어감"
        nb = copy.deepcopy(base)
        cell = [c for c in nb["cells"] if c["cell_type"] == "code" and "SAVE_TOP_K = " in src(c)]
        assert len(cell) == 1
        s = src(cell[0])
        for k, v in [("MODE", '"full"'), ("RUN_NAME", f'"{cfg["run"]}"'), ("EXTRA_ARGS", f'"{cfg["extra"]}"'),
                     ("SAVE_TOP_K", "3"), ("SAVE_PRETRAINED", "False"), ("MAX_EPOCHS", str(cfg["max_epochs"])),
                     ("TIME_BUDGET_H", f'{cfg["budget"]:.1f}')]:
            s = setp(s, k, v)
        cell[0]["source"] = s.splitlines(keepends=True)
        folder = f"{a.out}/{name}"
        os.makedirs(folder, exist_ok=True)
        for old in glob.glob(f"{folder}/*.ipynb"):
            os.remove(old)
        path = f"{folder}/{cfg['kernel']}.ipynb"
        with open(path, "w", encoding="utf-8", newline="\n") as f:
            json.dump(nb, f, ensure_ascii=False, indent=1); f.write("\n")
        for i, c in enumerate(json.load(open(path, encoding="utf-8"))["cells"]):
            if c["cell_type"] == "code":
                compile(src(c), f"{name}#cell{i}", "exec")
        meta = {"id": f"{USER}/{cfg['kernel']}", "title": cfg["kernel"], "code_file": f"{cfg['kernel']}.ipynb", "language": "python",
                "kernel_type": "notebook", "is_private": True, "enable_gpu": True, "enable_tpu": False, "enable_internet": True,
                "keywords": ["gpu"], "dataset_sources": cfg["data"], "kernel_sources": [], "competition_sources": [],
                "model_sources": [], "docker_image": IMAGE, "machine_shape": "NvidiaTeslaT4"}
        with open(f"{folder}/kernel-metadata.json", "w", encoding="utf-8") as f:
            json.dump(meta, f, indent=2)
        print(f"{name:16s} 커널 {cfg['kernel']:20s} {cfg['run']:30s} 데이터 {cfg['data'][0]}\n"
              f"{'':16s} 시퀀스 ≈{p['seq']} · {p['spe']} 스텝/에폭 · 에폭 {p['per_min']:.0f}분 · {p['epochs']} 에폭 ≈ {p['hours']:.1f} h"
              f"(예산 {cfg['budget']} h 에 {p['fit']} 에폭까지) · 최적화 {p['opt_steps']} 스텝\n{'':16s} # {cfg['note']}")
    print(f"\n올리기: cd <폴더> && python -m kaggle kernels push   (한 커널에 하나씩 — 같은 커널 이름 둘은 동시에 못 돎)")


if __name__ == "__main__":
    main()
