"""설정 읽기 — `base:` 로 틀(템플릿)을 물려받고, 장소 설정에는 바뀌는 것만 적는다.

    # recon/configs/corridor.yaml
    base: templates/indoor_rtabmap.yaml   # 이 파일 기준 상대 경로 (틀이 또 base 를 가져도 됨)
    name: corridor
    input: {root: ~/recon_data/corridor_v0.1}
    sky: null                            # null = 틀에 있던 단계를 끔 (키를 지움)

합치는 규칙: dict 는 키마다 재귀로 덮어씀, 그 밖(list · 숫자 · 문자열)은 통째로 바꿈, null 은 키 삭제.
다 합친 뒤 남은 null 도 지움 — 틀에 `cars: null` 처럼 꺼 둔 단계는 '키 없음'과 같게 (각 단계는 키가 있으면 켜짐).
"""
import os

import yaml


def merge(base, over):
    out = dict(base)
    for k, v in over.items():
        if v is None:
            out.pop(k, None)
        elif isinstance(v, dict) and isinstance(out.get(k), dict):
            out[k] = merge(out[k], v)
        else:
            out[k] = v
    return out


def prune(c):
    if isinstance(c, dict):
        return {k: prune(v) for k, v in c.items() if v is not None}
    if isinstance(c, list):
        return [prune(v) for v in c]
    return c


def load(path, _seen=None):
    """설정 파일 → dict (base 를 물려받고 null 을 지운 것)."""
    cfg = _load(path, _seen)
    return cfg if _seen else prune(cfg)


def _load(path, _seen=None):
    path = os.path.abspath(os.path.expanduser(path))
    seen = (_seen or set()) | {path}
    cfg = yaml.safe_load(open(path, encoding="utf-8")) or {}
    b = cfg.pop("base", None)
    if b:
        bp = os.path.normpath(os.path.join(os.path.dirname(path), os.path.expanduser(b)))
        if bp in seen:
            raise SystemExit(f"config base 가 돌고 돎: {bp}")
        cfg = merge(_load(bp, seen), cfg)
    return cfg
