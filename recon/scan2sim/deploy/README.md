# scan2sim 배포 — 팀 ThinkPad(리눅스, GPU 없음)에 올리기

아이폰 촬영본(Stray Scanner zip)을 브라우저로 올리면 이 컴퓨터가 처리해 Isaac Sim용 USD 묶음을 돌려주는 서버(`recon/scan2sim/web/server.py`)를 설치하고, Cloudflare 임시 링크로 밖에서 들어오게 함.

이 문서는 ThinkPad에서 Claude Code로 따라 해도 되게 썼음. 명령은 모두 레포 맨 위 폴더에서 실행함.

## 스크립트

| 파일 | 하는 일 |
|---|---|
| `install.sh` | 파이썬 3.12 확인 → 가상 환경 `~/scan2sim_env` → `recon/requirements.txt` 설치 → 불러오기 확인 → cloudflared 내려받기(`~/.local/bin`) → 서버를 잠깐 켜서 응답 확인. 여러 번 돌려도 됨 |
| `start.sh` | 서버와 터널을 뒤에서 켜고 접속 주소 · 접속 코드를 보여 줌. 서버가 죽으면 다시 켬 |
| `status.sh` | 켜져 있는지 · 주소 · 접속 코드 · 코드가 든 링크와 QR(터미널 + `~/scan2sim_run/qr.png`) · 기다리는 작업 수 · 예시 파일 수 · 작업 폴더 용량과 디스크 여유 |
| `stop.sh` | 서버와 터널을 끔 |
| `check.py` | 켜진 서버에 파일을 올려 끝까지 돌려 보고 결과 4종을 받아 봄(화면과 같은 순서). 마지막 줄 `CHECK_OK` / `CHECK_FAIL` |
| `common.sh` | 위 스크립트가 함께 쓰는 값. 직접 실행하지 않음 |

## 다시 켜진 뒤 알아서 켜지게 (`autostart.sh`)

ThinkPad가 꺼졌다 켜지면 서버와 터널도 꺼져 있음. 알아서 켜지게 하려면 한 번만:

```bash
S2S_TTL_HOURS=0 S2S_KEEP=-1 bash recon/scan2sim/deploy/autostart.sh on     # 이때 준 S2S_* 값이 같이 적힘
bash recon/scan2sim/deploy/autostart.sh status                            # 넣어 둔 줄 보기
bash recon/scan2sim/deploy/autostart.sh off                               # 빼기
```

- 사용자 crontab에 `@reboot` 한 줄을 넣음(sudo 없음, 다른 줄은 그대로 둠). 다시 켜지면 30초 뒤 `start.sh`를 부르고 `~/scan2sim_run/boot.log`에 적음.
- 접속 코드는 그대로이고, **터널 주소는 바뀜** — 다시 켜진 뒤 `status.sh`로 새 주소를 봄.
- 값을 바꾸려면 `on`을 새 값으로 다시 부름(줄이 바뀜). `S2S_CODE`는 줄에 적지 않음.
- 확인한 것(11.26, server207): 넣기 · 빼기 · 다른 줄이 남는 것, 넣은 줄의 명령을 깨끗한 환경에서 돌려 서버가 뜨는 것.
- **ThinkPad에서 껐다 켜 봄(11.27, `b9b3e7b`): 됨.** 부팅 19:58 → cron이 그 줄을 돌림 → 31초 뒤 서버(waitress, `--ttl-hours 0 --keep -1` 그대로) → 35초 뒤 터널 주소. 터널은 한 번에 붙었고 터널로 `/api/info`가 200. 접속 코드는 그대로 · 주소는 바뀜 · 작업 폴더 25건 그대로.
  - 이 ThinkPad는 자동 로그인이 켜져 있어 부팅 2초 만에 로그인 상태가 됨 — **로그인 없이도 켜지는지는 가리지 못했음**(cron 줄은 로그인 세션과 따로 돌고 와이파이 연결은 모든 사용자용이라 될 것으로 보이나 추정).
  - **절전 막기(`systemd-inhibit`)는 붙지 않음** — cron에서는 로그인 세션 밖이라 막힘(추정). `start.sh`는 안 되면 그것 없이 켬. 지금 잠들지 않는 것은 덮개 닫기 무시와 사용자 세션의 '가만히 둬도 잠들지 않음' 설정 덕분이므로, 그 둘을 바꾸면 안 됨. 손으로 `start.sh`를 부르면 절전 막기가 붙음.

## 순서

```bash
git clone git@github.com:JP-Avengers/FUTURE-DRIVE-PROJECT.git && cd FUTURE-DRIVE-PROJECT    # 이미 있으면 git pull
bash recon/scan2sim/deploy/install.sh
bash recon/scan2sim/deploy/start.sh
~/scan2sim_env/bin/python recon/scan2sim/deploy/check.py <촬영본.zip>                         # 이 컴퓨터 안에서
~/scan2sim_env/bin/python recon/scan2sim/deploy/check.py <촬영본.zip> --url <터널 주소>       # 터널을 지나서
```

그다음 아이폰 브라우저로 터널 주소에 들어가 접속 코드를 넣고 같은 촬영본을 올려 봄.

저장소는 통째로 받아야 함. 엔진이 `recon/` 밖의 `sim/nav2/make_map.py`를 쓰기 때문에 `recon/`만 복사하면 지도 · 진단 단계가 안 됨(`install.sh`가 확인함).

자동 시험(약 5~8분, 같은 입력을 두 번 돌려 결과가 같은지도 봄):

```bash
cd recon && PY=~/scan2sim_env/bin/python bash scan2sim/tests/run.sh
```

## 사람이 해야 하는 것 (스크립트 · Claude Code가 하지 않음)

- **sudo가 필요한 설치.** `install.sh`는 sudo를 쓰지 않음. 파이썬 3.12나 venv가 없으면 필요한 명령을 알려 주고 멈춤(우분투: `sudo apt install python3.12 python3.12-venv`). 이 명령은 사람이 직접 실행함.
- **덮개를 닫아도 잠들지 않게 하기.** `start.sh`는 가만히 둘 때 잠드는 것만 막음(`systemd-inhibit`). 덮개 설정은 시스템 설정에서 바꿈. 시연 중에는 전원을 꽂고 덮개를 열어 둠.
- **접속 주소와 코드 전달.** 터널 주소는 켤 때마다 바뀜. `status.sh`가 보여 주는 주소와 코드를 팀에 알림. 링크(`…/?code=<코드>`)와 QR에는 코드가 들어 있으므로 저장소 · 공개 자료에 남기지 않음. 화면이 링크의 코드를 알아서 채우는 것은 아직 안 됨(A) — 그때까지는 들어와서 코드를 직접 넣음.
- **예시 파일 넣기.** `~/scan2sim_samples` 폴더를 만들고 `.zip` · `.glb`를 넣은 뒤 서버를 다시 켜면 `GET /api/samples`에 나옴. 파일 이름은 영문 · 숫자 · `._-`만(80자 안). 화면에 보일 이름은 그 폴더의 `samples.json`에 `{"파일 이름": "이름표"}`로 적음. 사람 얼굴이 찍힌 촬영본은 넣지 않음.

## 값 바꾸기

실행할 때 환경 변수로 줌. 네 스크립트에 같은 값을 줘야 함.

| 변수 | 기본 | 뜻 |
|---|---|---|
| `S2S_ENV` | `~/scan2sim_env` | 가상 환경 |
| `S2S_JOBS` | `~/scan2sim_jobs` | 올린 파일과 결과. 레포 밖에 둘 것 |
| `S2S_RUN` | `~/scan2sim_run` | 로그(`server.log` · `tunnel.log`) · 프로세스 번호 · 접속 코드 |
| `S2S_PORT` | `8000` | 서버 포트(이 컴퓨터 안에서만 열림) |
| `S2S_TUNNEL` | `1` | `0`이면 터널 없이 이 컴퓨터에서만 |
| `S2S_CODE` | 처음 켤 때 6자리로 만듦 | 접속 코드. 한 번 정하면 `~/scan2sim_run/code`에 남아 계속 씀 |
| `S2S_SAMPLES` | `~/scan2sim_samples` | 예시 파일 폴더. 있을 때만 씀 |
| `S2S_MIN_FREE_GB` | `20` | 디스크 여유가 이보다 적으면 새 작업을 거절함(507). 디스크가 작은 컴퓨터면 낮춤 — `start.sh`가 켤 때 여유가 기준보다 적으면 알려 줌 |
| `S2S_TIMEOUT_MIN` | 서버 기본 20 | 한 건의 처리 제한 시간(분). 넘으면 멈추고 실패로 표시 |
| `S2S_MAX_WAIT` | 서버 기본 5 | 기다리는 작업이 이만큼이면 새 작업을 거절함(429) |
| `S2S_TTL_HOURS` | 서버 기본 24 | 끝난 지 이만큼 지난 작업 폴더를 통째로 지움. 미리 끝내 둔 결과를 시연 날까지 두려면 늘리거나 `0`(지우지 않음) |
| `S2S_KEEP` | 서버 기본 5 | 올린 파일 · 중간 파일까지 남겨 둘 최근 작업 수. 더 오래된 끝난 작업은 결과만 남김. `-1`이면 올린 파일을 지우지 않음(같은 촬영본을 다시 돌릴 일이 있으면) |
| `S2S_CODE_TRIES` | 서버 기본 30 | 같은 주소에서 10분 안에 접속 코드를 이만큼 틀리면 그 주소를 5분 동안 막음(429). `0`이면 세지 않음. 같은 와이파이의 사람들은 밖에서 보면 한 주소라 함께 막힘 — ThinkPad 자체 화면(127.0.0.1)은 따로 셈. 터널이 방문자 주소를 알려 주는지는 `status.sh`의 "방문자 주소 … 로 구분"으로 봄(터널로 한 번 들어온 뒤). 11.27 ThinkPad의 실제 터널에서 `CF-Connecting-IP 로 구분`으로 나옴. "구분 안 됨"이면 터널로 온 모두와 ThinkPad 화면이 한 주소로 세어지므로 `0`으로 끌 것 |
| `S2S_QR` | `1` | `0`이면 `status.sh`가 QR을 그리지 않음 |

## ThinkPad에서 확인해서 알려 줄 것

아래는 다른 컴퓨터에서는 확인할 수 없어서 아직 모르는 것임. 결과를 `integration_log.md`에 적음.

1. `install.sh`가 끝까지 가는지, 걸린 시간.
2. `check.py`를 이 컴퓨터 안에서 돌린 처리 시간(17초 예시 촬영본, 그리고 1~2분 촬영본).
3. `check.py --url <터널 주소>`가 통과하는지와 올리는 속도. 조각 크기는 화면과 같은 8 MB임. Cloudflare 임시 터널은 요청 하나의 크기를 제한할 수 있으므로 `--chunk-mb 64`, `--chunk-mb 128`도 돌려 어디서 막히는지 봄(막혀도 8 MB가 통과하면 문제없음).
4. 터널을 지나 `preview.glb` · `bundle.zip`을 받는 데 걸린 시간(`check.py`가 보여 줌).
5. 큰 파일을 터널로 올리는 데 걸린 시간. `check.py`가 올리기 시간과 MB/s를 보여 줌. 5분 촬영 zip이 약 960 MB라 상향 20 Mbps면 6분이 넘음(계산값) — 시연할 네트워크에서 실제 값을 잴 것. 30 fps · 15 fps로 찍은 촬영본이 있으면 둘 다.
6. `tests/run.sh`의 끝 줄(`TESTS_OK` / `TESTS_FAIL`)과 걸린 시간.
7. 아이폰 사파리에서 Stray Scanner가 내보낸 zip을 골라 올릴 수 있는지, 올리는 중에 화면을 끄거나 다른 앱으로 가면 어떻게 되는지.

## 문제가 생기면

- 서버가 안 뜸 → `tail -50 ~/scan2sim_run/server.log`. 포트가 쓰이고 있으면 `S2S_PORT=8010`으로 `start.sh` · `status.sh` · `stop.sh`를 실행.
- 터널 주소가 안 나옴 → `tail -50 ~/scan2sim_run/tunnel.log`. 밖으로 나가는 7844 포트(TCP · UDP)가 막힌 네트워크에서는 터널이 안 붙음. 다른 네트워크(휴대폰 핫스팟)로 바꿔 봄.
- 작업이 `failed` → `~/scan2sim_jobs/<작업 번호>/run.txt`에 엔진 출력, `out/report.md`에 단계별 결과가 있음.
- 디스크가 참 → 오래된 작업은 서버가 알아서 줄이고 지움(아래 한계 참고). 그래도 차면 작업 폴더를 지움: `rm -r ~/scan2sim_jobs/<작업 번호>`.
- 올리려는데 "서버 디스크 여유가 모자라"(507) → `status.sh`의 디스크 여유와 기준을 봄. 자리를 비우거나 `S2S_MIN_FREE_GB`를 낮춰 다시 켬.
- "기다리는 작업이 N건이라 지금은 받을 수 없음"(429) → 줄이 줄어들면 다시 올림. `status.sh`에 기다리는 작업 수가 나옴.
- 작업이 "처리가 N분을 넘어 멈춤"으로 실패 → 촬영본이 길거나 넓음. 더 짧게 찍거나 `S2S_TIMEOUT_MIN`을 늘려 다시 켬.

## 알려진 한계 (11.25)

- 한 번에 한 건만 처리하고 나머지는 줄을 세움.
- 서버가 다시 켜지면 기다리던 작업 · 돌던 작업은 줄에 다시 들어가 처음부터 돎(작업마다 한 번만). 올리던 파일은 화면이 받은 자리부터 이어 올림(끊기면 8번까지 다시 시도).
- 작업 폴더: 끝난 작업은 올린 zip 을 풀어 둔 사본을 바로 지우고, 최근 5건보다 오래된 작업은 올린 파일 · 중간 파일도 지워 결과만 남김(실제 촬영본 한 건 1.1 GB → 86 MB). 끝난 지 24시간(`S2S_TTL_HOURS`)이 지난 작업과 그만큼 멈춰 있는 올리다 만 작업은 폴더째 지움 — 그 뒤에는 결과 화면 주소(`…/#<작업 번호>`)도 열리지 않음.
- waitress로 켬(없으면 Flask 내장 서버). 시연용(동시에 몇 명)임.
- `bundle.zip`은 맨 위가 `bundle/` 폴더 하나이고 보고서 · 결함 목록도 그 안에 있음(`sim/add_site.py`에 그대로 줄 수 있음 — `docs/isaac_check.md`). 11.25 전에 끝난 작업의 `bundle.zip`은 예전 구조 그대로임.
