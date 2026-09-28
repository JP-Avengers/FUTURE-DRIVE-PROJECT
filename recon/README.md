# recon/ — Mesh / USD

- **담당**: C · 이진우
- **목적**: Point Cloud 를 정제해 Open3D 메쉬로 만들고, 경량화·Collider 를 거쳐 Isaac Sim 용 USD 로 변환한다.

## 간트 작업

| 작업 | 기간 |
| --- | --- |
| Point Cloud 정제 · Mesh v0 | 10.05–10.23 |
| 경량화 · Collider · USD 최종 | 10.24–11.16 |

## 입력 · 출력

- **받는 것**: B 의 포인트클라우드 · [`b2c_map.md`](../docs/interfaces/b2c_map.md)
- **넘기는 것**: USD → D · [`docs/interfaces/c2d_usd.md`](../docs/interfaces/c2d_usd.md) (**드래프트 USD 중간 납기 포함**)

## 환경

공용 노트북 · Python 3.12 venv `~/o3d_env` — 버전은 [`requirements.txt`](requirements.txt) 로 고정.

```bash
python3 -m venv ~/o3d_env && ~/o3d_env/bin/pip install -r recon/requirements.txt
```

> 메쉬·USD 바이너리(`*.ply` `*.obj` `*.usdc` …)는 커밋하지 않습니다 — [`MANIFEST.md`](../MANIFEST.md) 에 기록.

## 실행 방법

레포 루트에서. 출력은 레포 밖(`--out`, 기본 `~/jinwoo/out`)에 쓴다.

```bash
# 1) 샘플 점군(Open3D PLYPointCloud)으로 전체 리허설: 정제 → Poisson → 경량화 → Collider → OBJ  (약 4초)
~/o3d_env/bin/python recon/rehearsal/rehearsal.py --out ~/jinwoo/out
# 2) OBJ → USD (/World/VisualMesh + /World/ColliderMesh, Z-up · m)
~/o3d_env/bin/python recon/rehearsal/to_usd.py --out ~/jinwoo/out
```

## 알려진 한계

- `rehearsal.py` 의 Collider 는 점군 **전체의 convex hull 하나**라 실내 샘플 검증용이다. 실제 주차장 씬에서는 로봇이 볼록 덩어리 안에 갇히므로, 지형은 삼각형 메쉬 충돌·장애물은 convex decomposition 으로 교체 예정 ([`c2d_usd.md`](../docs/interfaces/c2d_usd.md) Collider 방식과 함께 D 와 합의).
