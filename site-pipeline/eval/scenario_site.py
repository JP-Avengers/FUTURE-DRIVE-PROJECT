#!/usr/bin/env python3
"""시나리오 yaml 에서 장소에 딸린 값을 찾는 한 곳 (11.17, D) — 씬 · 방향별 기준선 · Nav2 지도 · 바닥 메쉬 · 출발 방향.

일괄 실행(sim/nav2/scenario_batch.sh · eval/xm_batch.sh · sim/dagger_batch.sh)과 하네스(eval/xm_route.py), sim/make_obstacles.py 가
같은 규칙을 쓴다. 예전에는 스크립트마다 주차장 r3 값(기준선 · 지도 · 메쉬 이름)이 박혀 있어, 다른 장소는 링크나 환경 변수로 돌려야 했다.

    python3 eval/scenario_site.py <시나리오.yaml> <항목> [--direction cw|ccw] [--scene <씬 루트>]

항목과 찾는 순서(앞의 것이 먼저):
    scene       시나리오의 scene
    route       구간 시나리오이거나 방향이 없으면 시나리오의 route.
                바퀴 + 방향이면 routes.<방향> → route 파일 이름 끝의 _cw · _ccw 를 그 방향으로 바꾼 파일 → (cw 면) route 그대로
    route_yaml  위 route 와 같은 이름의 .yaml(eval/routes/make_refline.py 가 CSV 와 함께 쓰는 파일)
    start_yaw   route_yaml 의 start.yaw_deg
    nav2_map    시나리오의 nav2_map → data/nav2/<씬 파일 이름>/map.yaml
    mesh        시나리오의 mesh → data/usd/<씬 파일 이름>/usd/ 안의 *_mesh.usdc 하나(링크가 아닌 파일이 먼저)
--scene 을 주면 nav2_map · mesh 는 그 씬 기준으로 찾는다(FD_SCENE 으로 다른 판 씬에 같은 시나리오를 돌릴 때).
경로는 레포 루트 기준 상대 경로 그대로 돌려준다. 못 찾으면 이유를 stderr 에 적고 코드 2.
"""
import argparse
import glob
import os
import re
import sys

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ITEMS = ("scene", "route", "route_yaml", "start_yaw", "nav2_map", "mesh")


class SiteError(Exception):
    """장소 값을 못 찾음 — 메시지에 무엇을 어디서 찾았는지 적는다."""


def load(path):
    import yaml
    with open(path, encoding="utf-8") as f:
        return yaml.safe_load(f) or {}


def is_segment(sc):
    """구간 시나리오(goal 에 s) — sim/nav2/nav2_route.py · eval/xm_route.py 와 같은 판별."""
    return "s" in (sc.get("goal") or {})


def scene_of(sc, scene=None):
    s = scene or sc.get("scene") or ""
    if not s:
        raise SiteError("시나리오에 scene 이 없음")
    return s


def scene_stem(sc, scene=None):
    """씬 루트 파일 이름에서 확장자를 뗀 것 — data/usd · data/nav2 아래 폴더 이름과 같다(parking_v0.8_r3 · corridor_v0.2)."""
    return os.path.splitext(os.path.basename(scene_of(sc, scene)))[0]


def route_for(sc, direction=None):
    """주행이 따르는 기준선 CSV. 구간 시나리오는 방향과 상관없이 시나리오의 route."""
    base = sc.get("route")
    if direction is None or is_segment(sc):
        if not base:
            raise SiteError("시나리오에 route 가 없음")
        return base
    routes = sc.get("routes") or {}
    if direction in routes:
        return routes[direction]
    if not base:
        raise SiteError("시나리오에 route 가 없음")
    m = re.match(r"^(.*)_(cw|ccw)(\.[A-Za-z0-9]+)$", base)
    if m:
        return f"{m.group(1)}_{direction}{m.group(3)}"
    if direction == "cw":
        return base
    raise SiteError(f"{direction} 방향 기준선을 알 수 없음 — route 이름이 _cw 로 끝나지 않고 routes.{direction} 도 없음: {base}")


def route_yaml_for(sc, direction=None):
    return os.path.splitext(route_for(sc, direction))[0] + ".yaml"


def start_yaw_for(sc, direction=None, root=REPO):
    p = route_yaml_for(sc, direction)
    try:
        return float(load(os.path.join(root, p))["start"]["yaw_deg"])
    except (OSError, KeyError, TypeError, ValueError) as e:
        raise SiteError(f"{p} 에서 start.yaw_deg 를 못 읽음({e.__class__.__name__})")


def nav2_map_for(sc, scene=None):
    if sc.get("nav2_map") and not scene:
        return sc["nav2_map"]
    return os.path.join("data/nav2", scene_stem(sc, scene), "map.yaml")


def mesh_for(sc, scene=None, root=REPO):
    """바닥 높이를 잴 충돌 메쉬 — 씬과 같은 판의 맵 폴더에서 찾는다."""
    if sc.get("mesh") and not scene:
        return sc["mesh"]
    d = os.path.join("data/usd", scene_stem(sc, scene), "usd")
    found = sorted(glob.glob(os.path.join(root, d, "*_mesh.usdc")))
    real = [p for p in found if not os.path.islink(p)]
    if not real:   # 링크뿐이면 가리키는 파일이 같은 것끼리 하나로(이름순 첫 번째)
        first = {}
        for p in found:
            first.setdefault(os.path.realpath(p), p)
        real = sorted(first.values())
    if len(real) == 1:
        return os.path.join(d, os.path.basename(real[0]))
    if not real:
        raise SiteError(f"바닥 메쉬를 못 찾음 — {d}/ 에 *_mesh.usdc 가 없음(시나리오에 mesh 를 적거나 --mesh 로 지정)")
    raise SiteError(f"바닥 메쉬가 여러 개 — {', '.join(os.path.basename(p) for p in real)} (시나리오에 mesh 를 적거나 --mesh 로 지정)")


def get(sc, item, direction=None, scene=None, root=REPO):
    if item == "scene":
        return scene_of(sc, scene)
    if item == "route":
        return route_for(sc, direction)
    if item == "route_yaml":
        return route_yaml_for(sc, direction)
    if item == "start_yaw":
        return start_yaw_for(sc, direction, root)
    if item == "nav2_map":
        return nav2_map_for(sc, scene)
    if item == "mesh":
        return mesh_for(sc, scene, root)
    raise SiteError(f"모르는 항목: {item} — {', '.join(ITEMS)} 중 하나")


def main(argv=None):
    ap = argparse.ArgumentParser(description="시나리오 yaml 에서 장소에 딸린 값을 찾아 한 줄로 찍음")
    ap.add_argument("scenario")
    ap.add_argument("item", choices=ITEMS)
    ap.add_argument("--direction", choices=["cw", "ccw"], help="바퀴 시나리오의 방향(route · route_yaml · start_yaw)")
    ap.add_argument("--scene", help="시나리오의 scene 대신 쓸 씬 루트(nav2_map · mesh 를 이 씬 기준으로)")
    a = ap.parse_args(argv)
    try:
        sc = load(a.scenario if os.path.isabs(a.scenario) or os.path.exists(a.scenario) else os.path.join(REPO, a.scenario))
        print(get(sc, a.item, a.direction, a.scene or None))
    except SiteError as e:
        print(f"scenario_site: {a.scenario}: {e}", file=sys.stderr)
        return 2
    except OSError as e:
        print(f"scenario_site: {a.scenario}: 파일을 못 읽음({e.__class__.__name__})", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())
