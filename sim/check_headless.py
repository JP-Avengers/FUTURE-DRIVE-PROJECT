"""Isaac Sim 헤드리스 기동 검증 — GPU 0 고정.

실행 (server207):
    OMNI_KIT_ACCEPT_EULA=YES $CD1/envs/isaacsim_env/bin/python sim/check_headless.py

기대: "OK" 출력 후 종료 코드 0. 첫 실행은 셰이더 캐시 생성으로 수십 초~수십 분.

주의: CUDA_VISIBLE_DEVICES 를 함께 걸지 말 것 — 9.27 실측에서 종료 시 abort(EXIT=134).
GPU 1 에 ~220MiB 컨텍스트가 잡히는 것은 정상 (렌더 Active 는 GPU 0 하나).
"""
from isaacsim import SimulationApp

app = SimulationApp({"headless": True, "active_gpu": 0, "physics_gpu": 0, "multi_gpu": False})
print("OK", flush=True)
app.close()
