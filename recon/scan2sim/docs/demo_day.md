# 발표 당일 순서 (D)

12/7 발표와 12/5 리허설에서 서버를 켜고 시연하는 순서. `implementation.md` 11장을 명령까지 풀어 쓴 것.
스크립트 설명은 `deploy/README.md`, 명령은 모두 레포 맨 위 폴더에서 실행함.

**이 문서의 순서는 아직 ThinkPad에서 돌려 보지 않았음(11.25).** 서버(server207)에서 터널 없이 켜기 · 예시 파일 한 건 · `status.sh`의 링크와 QR · 끄기만 확인함. 터널을 지나는 것과 폰에서 올리는 것은 M1(11/27)에 확인하고 여기를 고침.

## 전날까지

1. ThinkPad에서 `git pull` → `bash recon/scan2sim/deploy/install.sh`가 끝까지 감(새 패키지 `waitress` · `segno`가 들어감).
2. 예시 파일을 `~/scan2sim_samples`에 넣음 — 가상의 방 GLB 하나와 실제 촬영본 zip 하나. 사람 얼굴이 찍힌 촬영본은 넣지 않음. 화면에 보일 이름은 `~/scan2sim_samples/samples.json`에 적음.

    ```bash
    mkdir -p ~/scan2sim_samples && cd recon
    ~/scan2sim_env/bin/python scan2sim/samples/make_room.py --out /tmp/room && cp /tmp/room/room.glb ~/scan2sim_samples/
    ```

3. 덮개를 닫아도 잠들지 않게 시스템 설정을 바꿈(`start.sh`는 가만히 둘 때 잠드는 것만 막음). 충전기를 챙김.
4. 디스크 여유를 봄(`status.sh`). 기준(기본 20 GB)보다 적으면 서버가 새 작업을 받지 않으므로 자리를 비우거나 `S2S_MIN_FREE_GB`를 낮춤.
5. 미리 끝내 둘 결과 두 건(GLB · Stray)은 **당일 아침에** 만듦. 끝난 작업은 24시간 뒤 지워지므로 전날 만든 것은 발표 때 없을 수 있음. 전날 만들어 두려면 `S2S_TTL_HOURS=72`로 켬.
6. 터널이 안 될 때 보여 줄 녹화 영상을 노트북에 넣어 둠.

## 당일 — 켜기 (발표 30분 전까지)

1. ThinkPad를 켜고 충전기와 네트워크를 연결함. 발표 장소 네트워크가 안 되면 휴대폰 핫스팟.
2. 켬. 접속 주소 · 접속 코드 · 링크 · QR이 나옴.

    ```bash
    bash recon/scan2sim/deploy/start.sh
    ```

    - 터널 주소는 켤 때마다 바뀜. 서버만 다시 켜질 때는 주소가 그대로지만, 터널이 다시 켜지면 바뀜 → `status.sh`로 새 주소와 QR을 다시 봄.
    - QR 그림은 `~/scan2sim_run/qr.png`에도 남음. 발표 화면에 띄울 때 이 파일을 씀. **링크와 QR에는 접속 코드가 들어 있으므로 공개하는 발표 자료 · 저장소에는 넣지 않음.**
3. 다른 기기(폰)로 QR에 들어가 접속 코드를 넣고 예시 파일 한 건을 끝까지 돌려 봄: 결과 화면 → 지도의 결함 강조 → `bundle.zip` 내려받기.
4. 미리 끝내 둘 결과 두 건을 만들고 결과 화면 주소(`<터널 주소>/#<작업 번호>`)를 적어 둠.

    ```bash
    ~/scan2sim_env/bin/python recon/scan2sim/deploy/check.py ~/scan2sim_samples/room.glb
    ~/scan2sim_env/bin/python recon/scan2sim/deploy/check.py <촬영본.zip>
    ```

5. `bash recon/scan2sim/deploy/status.sh`로 한 번 더 봄: 서버 응답 · 터널 주소 · 기다리는 작업 0건 · 디스크 여유.

## 시연 중

- 한 번에 한 건만 처리하고 나머지는 줄을 섬. 기다리는 작업이 5건이면 새 작업을 받지 않고 "몇 분 뒤 다시"를 알려 줌.
- 한 건이 20분을 넘으면 멈추고 실패로 표시함. 그 뒤 작업은 그대로 이어짐.
- 청중이 올린 파일은 이 노트북에 저장되고 24시간 뒤 지워짐. 물어보면 그렇게 답함.
- 상태를 볼 때는 `status.sh`, 자세히는 `tail -f ~/scan2sim_run/server.log`.

## 안 될 때

| 증상 | 할 일 |
|---|---|
| 터널 주소가 안 나옴 | `tail -50 ~/scan2sim_run/tunnel.log`. 밖으로 나가는 7844 포트(TCP · UDP)가 막힌 네트워크면 핫스팟으로 바꾸고 `stop.sh` → `start.sh` |
| 주소가 바뀜(터널이 다시 켜짐) | `status.sh`로 새 QR을 띄움. 미리 적어 둔 결과 화면 주소도 앞부분을 새 주소로 바꿈(작업 번호는 그대로) |
| 폰에서 올리기가 느리거나 끊김 | 화면이 받은 자리부터 이어 올림(8번까지). 그래도 안 되면 예시 파일 단추나 미리 끝내 둔 결과 화면으로 넘어감 |
| 서버가 응답하지 않음 | 죽으면 2초 뒤 스스로 다시 켜짐. 돌던 작업은 줄에 다시 들어감. 1분 넘게 안 되면 `stop.sh` → `start.sh` |
| 핫스팟으로도 터널이 안 됨 | `S2S_TUNNEL=0 bash recon/scan2sim/deploy/start.sh`로 켜고 노트북 화면(`http://127.0.0.1:8000`)으로 직접 시연 |
| 그것도 안 됨 | 녹화 영상 |

## 끝난 뒤

```bash
bash recon/scan2sim/deploy/stop.sh
rm -r ~/scan2sim_jobs ~/scan2sim_run/qr.png      # 청중이 올린 파일을 남기지 않음
```

## 아직 정해지지 않았거나 확인 안 된 것

- 화면이 링크의 `?code=`를 읽어 접속 코드를 채우는 것(A). 될 때까지는 QR로 들어와 코드를 직접 넣어야 하므로, 코드를 QR 옆에 같이 띄움.
- 화면의 예시 파일 단추(A). 서버 쪽 `GET /api/samples` · `POST /api/samples/<name>/jobs`는 있음.
- 발표 장소 네트워크에서 터널이 붙는지, 올리는 속도(M4 리허설에서 잼).
- ThinkPad의 처리 시간과 디스크 여유(M1에 잼).
