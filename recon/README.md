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

공용 노트북 · Python (open3d 등 — `requirements.txt` 로 고정 예정).

> 메쉬·USD 바이너리(`*.ply` `*.obj` `*.usdc` …)는 커밋하지 않습니다 — [`MANIFEST.md`](../MANIFEST.md) 에 기록.

## 실행 방법

(작성 예정 — 명령 한 줄이라도 동작하는 것부터 적어 주세요)
