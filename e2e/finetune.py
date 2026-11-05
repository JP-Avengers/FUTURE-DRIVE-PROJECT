#!/usr/bin/env python3
"""X-Mobility 파인튜닝 — 사전학습 체크포인트 + 이 프로젝트 교사 데이터(xm_v1 등).

X-Mobility 레포(NVlabs/X-Mobility 5837aea)의 train.py 와 같은 학습 루프에 세 가지를 고친 것:
  1. 가중치 로드 — 원본 train.py 는 XMobilityTrainer.load_from_checkpoint(strict=False) 라서
     (a) 체크포인트에 저장된 lr · weight_decay · scheduler_pct_start 가 gin 값을 덮어쓰고(파인튜닝 lr 이 조용히 무시됨)
     (b) 이름이 안 맞는 가중치가 아무 말 없이 무작위 초기화로 남는다.
     여기서는 gin 값으로 모델을 만들고 state_dict 만 넣은 뒤, 빠진(missing) · 남는(unexpected) 키를 출력한다.
     missing 이 하나라도 있으면 멈춘다(--allow-missing 으로만 통과). 우리 데이터엔 의미 분할이 없어
     ENABLE_SEMANTIC=False 이므로 사전학습 ckpt 의 model.semantic_decoder.* 가 unexpected 로 나오는 것은 정상.
  2. gin 바인딩 오타 — 원본은 skip_unknown=True 라 틀린 이름이 조용히 무시된다. 여기서는 train · evaluate_* 만 건너뛰고 나머지 모르는 이름이면 멈춤.
  3. 기록 — 출력 폴더에 run_meta.json(이 레포 · X-Mobility 레포 커밋, ckpt sha256, 데이터 경로, 명령, 키 대조 결과)과
     gin_operative.txt 를 남긴다(MANIFEST 의 checkpoint 행 출처 칸에 그대로 옮김).

사용(server207, X-Mobility 의존성이 깔린 python):
  python e2e/finetune.py --xm-repo ~/xmobility_test/X-Mobility \\
      -c e2e/configs/finetune_parking_r3.gin -d data/teacher/xm_v1 \\
      -p <x_mobility-nav2-semantic_action_path.ckpt> -o data/checkpoints/xm_ft_<버전>
  시험(몇 스텝만): --max-steps 20 --limit-val-batches 2 --gin "finetune.precision='32'"
W&B: 기본 WANDB_MODE=offline(출력 폴더 wandb/ 에 저장, 나중에 `wandb sync`). 온라인은 --wandb online + WANDB_API_KEY.
"""
import argparse
import datetime
import hashlib
import json
import math
import os
import statistics
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
# 사전학습 ckpt 에는 있지만 우리 설정(ENABLE_SEMANTIC=False)에는 없는 모듈 — unexpected 로 나와도 정상
EXPECTED_UNEXPECTED = ("model.semantic_decoder.",)
# base_train_config.gin 이 바인딩하지만 X-Mobility train.py · evaluate.py 에만 있는 configurable(여기선 import 안 함)
GIN_SKIP = ("train", "evaluate_observation", "evaluate_prediction")


def git_rev(path):
    try:
        rev = subprocess.check_output(["git", "-C", path, "rev-parse", "--short=12", "HEAD"], text=True).strip()
        dirty = subprocess.call(["git", "-C", path, "diff", "--quiet", "HEAD"]) != 0
        return rev + ("-dirty" if dirty else "")
    except Exception:  # noqa: BLE001 — git 이 없거나 레포가 아니면 기록만 비움
        return None


def sha12(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()[:12]


def parse_args():
    ap = argparse.ArgumentParser(description="X-Mobility 파인튜닝 (E)")
    ap.add_argument("--xm-repo", default=os.environ.get("XM_REPO"), help="X-Mobility 레포 경로(또는 XM_REPO)")
    ap.add_argument("-c", "--config-files", nargs="+", required=True)
    ap.add_argument("-d", "--dataset-path", required=True)
    ap.add_argument("-p", "--checkpoint-path", help="사전학습 ckpt(없으면 처음부터 학습 — 파인튜닝 아님)")
    ap.add_argument("-o", "--output-dir", required=True)
    ap.add_argument("--gin", action="append", default=[], help="gin 바인딩 덧붙이기(여러 번), 예: \"NUM_EPOCHS=3\"")
    ap.add_argument("--allow-missing", action="store_true", help="ckpt 에 없는 가중치가 있어도 진행")
    ap.add_argument("--devices", default="auto")
    ap.add_argument("--accelerator", default="auto")
    ap.add_argument("--ddp-backend", choices=["nccl", "gloo"], default="gloo" if os.name == "nt" else "nccl",
                    help="DDP 통신 백엔드 — Windows 는 nccl 이 없어 gloo(기본). X-Mobility 데이터 로더가 "
                         "DistributedSampler 를 써서 GPU 1장이어도 DDP 프로세스 그룹이 필요함")
    ap.add_argument("--max-steps", type=int, default=-1)
    ap.add_argument("--max-time", default=None, help="학습 시간 상한 \"DD:HH:MM:SS\" (Kaggle 12시간 세션 등). "
                    "닿으면 그 자리에서 멈추고 last.ckpt 저장 — lr 스케줄은 끝까지 안 내려가므로 에폭 수를 시간에 맞추는 게 우선")
    ap.add_argument("--limit-train-batches", type=float, default=None)
    ap.add_argument("--limit-val-batches", type=float, default=None)
    ap.add_argument("--save-top-k", type=int, default=3, help="val_loss 상위 몇 개를 남길지(+ last). 디스크가 작으면 1")
    ap.add_argument("--no-test", action="store_true", help="학습 뒤 test split 평가 건너뜀")
    ap.add_argument("--wandb", choices=["offline", "online", "disabled"], default="offline")
    ap.add_argument("--wandb-entity", default=None)
    ap.add_argument("--wandb-project", default="fd_xmobility_finetune")
    ap.add_argument("--wandb-run", default=None)
    return ap.parse_args()


def main():
    a = parse_args()
    if not a.xm_repo or not os.path.isfile(os.path.join(a.xm_repo, "train.py")):
        sys.exit("--xm-repo(또는 XM_REPO)에 X-Mobility 레포 경로를 주세요 (train.py 가 있는 폴더)")
    xm = os.path.abspath(os.path.expanduser(a.xm_repo))
    cfgs = [os.path.abspath(c) for c in a.config_files]
    data = os.path.abspath(a.dataset_path)
    out = os.path.abspath(a.output_dir)
    ckpt = os.path.abspath(a.checkpoint_path) if a.checkpoint_path else None
    for sp in ("train", "val") + (() if a.no_test else ("test",)):
        if not os.path.isdir(os.path.join(data, sp)):
            sys.exit(f"데이터에 {sp}/ 없음: {data} — 먼저 python3 e2e/check_xm_dataset.py {data}")
    if os.path.isdir(os.path.join(out, "checkpoints")) and os.listdir(os.path.join(out, "checkpoints")):
        sys.exit(f"이미 체크포인트가 있음: {out}/checkpoints — 덮어쓰지 않습니다(버전을 올리세요)")
    os.makedirs(out, exist_ok=True)
    os.environ["WANDB_MODE"] = a.wandb
    os.environ.setdefault("WANDB_DIR", out)

    sys.path.insert(0, xm)
    import gin
    import pytorch_lightning as pl
    import torch
    from pytorch_lightning.callbacks.model_checkpoint import ModelCheckpoint
    from pytorch_lightning.loggers import WandbLogger
    from pytorch_lightning.strategies import DDPStrategy
    # gin 이 바인딩을 알아보도록 X-Mobility 모듈을 먼저 import (train.py 와 같은 순서)
    from model.dataset.isaac_sim_dataset import XMobilityIsaacSimDataModule  # noqa: F401
    from model.trainer import XMobilityTrainer

    @gin.configurable
    def finetune(epochs, precision, freeze_prefixes=()):
        return epochs, precision, tuple(freeze_prefixes)

    # include 'configs/base_train_config.gin' 은 파싱하는 동안만 X-Mobility 레포로 cd 해서 찾게 함 — gin 0.5.0 은 현재 폴더에서
    # 못 찾으면 'configs' 를 파이썬 패키지로 찾는데, sys.path 의 e2e/configs(패키지 아님)가 namespace 로 잡혀
    # gin/resource_reader.py:89 에서 TypeError. base 설정의 train · evaluate_* 는 X-Mobility train.py · evaluate.py 의
    # configurable 이라 여기선 등록 안 됨 → 그 이름만 건너뛰고, 나머지 모르는 이름(오타)은 그대로 멈춤.
    gin.add_config_file_search_path(xm)
    cwd = os.getcwd()
    os.chdir(xm)
    try:
        gin.parse_config_files_and_bindings(cfgs, a.gin, skip_unknown=list(GIN_SKIP))
    finally:
        os.chdir(cwd)
    epochs, precision, freeze = finetune()

    model = XMobilityTrainer()                # lr · weight_decay · pct_start = gin 값
    keys = None
    if ckpt:
        sd = torch.load(ckpt, map_location="cpu", weights_only=False)
        hp = sd.get("hyper_parameters", {})
        sd = sd.get("state_dict", sd)
        res = model.load_state_dict(sd, strict=False)   # 모양이 다른 키는 여기서 RuntimeError
        unexp_other = [k for k in res.unexpected_keys if not k.startswith(EXPECTED_UNEXPECTED)]
        keys = dict(loaded=len(sd) - len(res.unexpected_keys), missing=list(res.missing_keys),
                    unexpected_expected=len(res.unexpected_keys) - len(unexp_other), unexpected_other=unexp_other,
                    ckpt_hparams={k: hp[k] for k in ("lr", "weight_decay", "scheduler_pct_start") if k in hp})
        print(f"[ckpt] {ckpt}\n  넣은 키 {keys['loaded']} · missing {len(res.missing_keys)} · "
              f"unexpected {len(res.unexpected_keys)}(그중 의미 분할 등 예상분 {keys['unexpected_expected']})")
        print(f"  ckpt 하이퍼파라미터 {keys['ckpt_hparams']} → 이번 학습 lr {model.lr} · wd {model.weight_decay}")
        for k in res.missing_keys[:20]:
            print(f"  missing: {k}")
        for k in unexp_other[:20]:
            print(f"  unexpected(예상 밖): {k}")
        if res.missing_keys and not a.allow_missing:
            sys.exit("ckpt 에 없는 가중치가 있음 — 설정(gin)이 사전학습 때와 다른지 확인. 알고도 진행하려면 --allow-missing")
    else:
        print("[ckpt] 없음 — 처음부터 학습(파인튜닝 아님)")

    n_frozen = 0
    for name, prm in model.named_parameters():
        if freeze and name.startswith(freeze):
            prm.requires_grad = False
            n_frozen += prm.numel()
    n_train = sum(p.numel() for p in model.parameters() if p.requires_grad)
    print(f"[모델] 학습 파라미터 {n_train / 1e6:.1f} M · 고정 {n_frozen / 1e6:.1f} M (freeze_prefixes={list(freeze)})")

    meta = dict(started=datetime.datetime.now().isoformat(timespec="seconds"), argv=sys.argv,
                repo_rev=git_rev(REPO), xm_repo=xm, xm_rev=git_rev(xm), configs=cfgs, gin_extra=a.gin,
                dataset=data, checkpoint=ckpt, checkpoint_sha12=sha12(ckpt) if ckpt else None, keys=keys,
                epochs=epochs, precision=precision, freeze_prefixes=list(freeze), ddp_backend=a.ddp_backend,
                trainable_params=n_train, torch=torch.__version__, lightning=pl.__version__)
    rank0 = os.environ.get("LOCAL_RANK", "0") == "0"   # DDP 는 이 스크립트를 GPU 수만큼 다시 띄움
    if rank0:
        with open(os.path.join(out, "run_meta.json"), "w", encoding="utf-8") as f:
            json.dump(meta, f, ensure_ascii=False, indent=1)

    class StepTimer(pl.Callback):
        """학습 스텝 사이 시간(데이터 로딩 포함) — 남은 시간에 맞춰 에폭 수를 정하는 데 씀."""
        def __init__(self):
            self.starts = []

        def on_train_batch_start(self, trainer, pl_module, batch, batch_idx):
            self.starts.append(time.perf_counter())

        def step_time(self, skip=5):
            d = [b - a for a, b in zip(self.starts, self.starts[1:])][skip:]
            return statistics.median(d) if d else None

    timer = StepTimer()
    dm = XMobilityIsaacSimDataModule(dataset_path=data)
    logger = WandbLogger(entity=a.wandb_entity, project=a.wandb_project, name=a.wandb_run or os.path.basename(out),
                         save_dir=out, group="DDP", log_model=False)
    ckpt_cb = ModelCheckpoint(dirpath=os.path.join(out, "checkpoints"), save_top_k=a.save_top_k, monitor="val_loss",
                              mode="min", save_last=True)
    kw = {}
    for k in ("limit_train_batches", "limit_val_batches"):   # 1 이하 = 비율, 그보다 크면 배치 수(Lightning 규칙)
        v = getattr(a, k)
        if v is not None:
            kw[k] = int(v) if v > 1 else v
    trainer = pl.Trainer(max_epochs=epochs, max_steps=a.max_steps, precision=precision, sync_batchnorm=True,
                         accelerator=a.accelerator, devices=a.devices, strategy=DDPStrategy(find_unused_parameters=True, process_group_backend=a.ddp_backend),
                         callbacks=[pl.callbacks.ModelSummary(-1), pl.callbacks.LearningRateMonitor(), ckpt_cb, timer],
                         max_time=a.max_time, logger=logger, **kw)
    trainer.fit(model, datamodule=dm)

    if trainer.global_rank == 0:
        with open(os.path.join(out, "gin_operative.txt"), "w", encoding="utf-8") as f:
            f.write(gin.operative_config_str())
    test = None
    if not a.no_test:
        best = ckpt_cb.best_model_path or None
        res = trainer.test(model, datamodule=dm, ckpt_path=best or "last")
        test = dict(ckpt=best or "last", metrics=res[0] if res else None)
    if trainer.global_rank == 0:
        n_seq = len(dm.train_dataset)                      # 시퀀스 수(로더가 겹치지 않게 sequence_length 개씩 자름)
        per_rank = math.ceil(n_seq / trainer.world_size)   # DistributedSampler
        meta.update(step_time_s=timer.step_time(), train_sequences=n_seq,
                    steps_per_epoch=per_rank // dm.batch_size,   # drop_last=True, limit_train_batches 와 무관한 전체 값
                    world_size=trainer.world_size,
                    peak_gpu_mem_gb=round(torch.cuda.max_memory_allocated() / 2**30, 2) if torch.cuda.is_available() else None)
        meta.update(finished=datetime.datetime.now().isoformat(timespec="seconds"),
                    best_model_path=ckpt_cb.best_model_path, best_val_loss=float(ckpt_cb.best_model_score)
                    if ckpt_cb.best_model_score is not None else None, global_step=trainer.global_step, test=test)
        with open(os.path.join(out, "run_meta.json"), "w", encoding="utf-8") as f:
            json.dump(meta, f, ensure_ascii=False, indent=1, default=float)
        print(f"[끝] best {ckpt_cb.best_model_path} · val_loss {meta['best_val_loss']} · 기록 {out}/run_meta.json")


if __name__ == "__main__":
    main()
