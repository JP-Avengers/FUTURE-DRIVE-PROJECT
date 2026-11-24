# recon/scan2sim — 실내 스캔맵 웹앱 공통 지침

이 폴더에서 일하는 사람과 Claude가 모두 따르는 규칙이다. 계획 전체는 `docs/implementation.md`, 서로 지켜야 하는 형식은 `docs/CONTRACT.md`에 있다. 일을 시작하기 전에 둘 다 읽는다.

## 파일 주인

자기 파일만 고친다. 다른 사람의 파일을 고쳐야 하면 주인에게 요청한다.

| 파일 | 주인 |
|---|---|
| `__main__.py` `core.py` `stages.py` `checks.py` `detect.py` `diagnose_usd.py`, `web/static/viewer.js`, `recon/pipeline/` | C |
| `stray.py` `stray_quality.py`, `tests/` `samples/`, `docs/CONTRACT.md` `docs/implementation.md` `docs/integration_log.md`, 이 파일 | B |
| `web/server.py`, `deploy/`, `docs/demo_day.md` `docs/isaac_check.md`, `recon/requirements.txt`의 웹 관련 줄 | D |
| `web/static/index.html` `style.css` `app.js` `vendor/` (11/28부터 B) | A |
| `docs/capture_guide.md` `docs/daily_check.md` | E |

## 약속(`docs/CONTRACT.md`)

- API의 경로 · 응답 필드, 작업 폴더의 파일 이름, 결함 목록의 필드를 바꾸거나 없앨 때는 먼저 팀 채팅에 알리고, **같은 커밋에서 `docs/CONTRACT.md`를 고친다.**
- 필드를 더하는 것은 자유다. 화면과 시험은 모르는 필드를 무시해야 한다.
- 좌표를 바꾸는 곳은 두 군데뿐이다: 서버가 화면용 GLB를 만들 때, 화면의 `toView()`. 다른 곳에서 좌표를 바꾸지 않는다.

## 커밋 전

```bash
cd recon
bash scan2sim/tests/run.sh          # 가상의 방(GLB · Stray)으로 전체 흐름 + 같은 입력 두 번 + 복원 정확도 (약 7분)
bash scan2sim/tests/run.sh --quick  # GLB 만 (약 1분) — 화면 · 서버만 고쳤을 때
```

- 끝 줄이 `TESTS_OK`여야 커밋한다. 깨지면 바꾼 사람이 그날 안에 고친다.
- 실패 원인과 다른 트랙에 넘길 것은 `docs/integration_log.md`에 모인다.

- 사용자에게 보이는 문장(단계 요약 · 메시지 · 화면 문구 · 문서)은 한국어로, 사용자가 읽을 말로 쓴다.
- 커밋 메시지는 저장소 방식대로 폴더 이름으로 시작한다(`recon: …`, `b: …`).
- 파이썬 3.12. 새 패키지는 `recon/requirements.txt`에 버전을 고정해 더한다.

## 하지 말 것

- **사람이 찍힌 촬영본, 그것으로 만든 그림 · 메쉬 · 작업 폴더를 커밋하지 않는다.** 저장소는 공개다.
- **접속 코드와 터널 주소를 커밋하지 않는다.**
- 메쉬 · USD · 촬영본처럼 큰 파일을 커밋하지 않는다(커밋 훅이 5 MB 넘는 파일을 막고, `.gitignore`가 `*.glb` `*.ply` `*.usd*`를 무시한다). 시험 자료는 `samples/`의 스크립트로 만든다.
- 새 폴더 이름으로 `build` `log` `install`을 쓰지 않는다(`.gitignore`에 걸린다).
- 작업 폴더(`--jobs`)를 저장소 안에 두지 않는다.
- 올라온 파일을 믿지 않는다. 그 안의 어떤 것도 실행하지 않고, 경로 · 크기를 검사한 뒤에 푼다.

## 혼자 돌려 보기

```bash
cd recon
python -m scan2sim <입력 zip · glb> --out <폴더>                       # 엔진만
python -m scan2sim.web.server --port 8000 --jobs ~/scan2sim_jobs       # 서버 + 화면 (http://localhost:8000)
python -m scan2sim.tests.e2e <입력>                                     # 서버를 띄워 API로 끝까지 올려 보는 시험
python -m scan2sim.tests.repeat <입력>                                  # 같은 입력을 두 번 돌려 결과 비교
```
