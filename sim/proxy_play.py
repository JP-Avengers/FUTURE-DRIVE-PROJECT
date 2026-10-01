"""Proxy 씬을 열고 Play — sim/run_proxy.sh 가 부른다(직접 부를 일은 없음).

- stream: 스트리밍 앱(`isaacsim.exp.full.streaming`)의 `--exec` 로 앱 안에서 돈다. 씬을 비동기로 열고,
  뷰포트를 3인칭 카메라(ChaseCam)로 바꾼 뒤 Play. 화면의 Stop → Play 로 출발점에 돌아간다.
- headless: `envs/isaacsim_env/bin/python` 으로 돈다. SimulationApp 으로 열고 Play, FD_DURATION 초(0 이면 Ctrl+C 까지).
  Ctrl+C 는 Kit 가 받아 바로 끝낸다(종료 코드 0, 9.30) — PROXY_STOPPED 는 FD_DURATION 이 끝났을 때만 찍힌다.
- 공통: FD_LIDAR=1 이 아니면 3D LiDAR 렌더를 끈다(규격서 "수집·평가 설정에서는 끔"). 바꾸는 것은 세션 레이어뿐이라
  GUI 에서 저장해도 씬 파일(.usda)에는 남지 않는다.
- 로그 표지(grep 용): PROXY_LOADED · PROXY_SENSORS · PROXY_VIEW · PROXY_PLAYING · PROXY_TIMELINE · PROXY_RTF · PROXY_STOPPED
"""
import os
import time

SCENE = os.environ["FD_SCENE"]
MODE = os.environ.get("FD_PROXY_MODE", "headless")
LIDAR_ON = os.environ.get("FD_LIDAR", "0") == "1"
VIEW_RES = os.environ.get("FD_VIEW_RES", "")   # 예: 1280x720 — 비우면 앱 기본값
ROBOT = "/World/Nova_Carter_ROS"
RTF_EVERY = 30.0                               # s(실제 시간) — 로그에 RTF 를 찍는 간격


def say(msg):
    if MODE == "headless":
        print(msg, flush=True)
    else:  # Kit 앱 안의 print 는 콘솔에 안 나올 수 있어 경고 수준으로 찍는다(9.28 open_warmup 과 같음)
        import carb
        carb.log_warn(msg)


def set_sensors(stage):
    """LiDAR 렌더 프로덕트 노드의 inputs:enabled 를 세션 레이어에 쓴다 — sensors.usda 의 Hawk 렌더 끄기와 같은 방식."""
    from pxr import Sdf, Usd

    nodes = []
    for prim in Usd.PrimRange(stage.GetPrimAtPath(ROBOT)):
        if prim.GetTypeName() != "OmniGraphNode":
            continue
        if not str(prim.GetAttribute("node:type").Get() or "").endswith("IsaacCreateRenderProduct"):
            continue
        rel = prim.GetRelationship("inputs:cameraPrim")
        targets = rel.GetTargets() if rel else []
        if any(stage.GetPrimAtPath(t).GetTypeName() == "OmniLidar" or "lidar" in str(t).lower() for t in targets):
            nodes.append(prim)
    if not LIDAR_ON:
        with Usd.EditContext(stage, stage.GetSessionLayer()):
            for prim in nodes:
                prim.CreateAttribute("inputs:enabled", Sdf.ValueTypeNames.Bool).Set(False)
    say(f"PROXY_SENSORS lidar={'on' if LIDAR_ON else 'off'} nodes={[str(p.GetPath()) for p in nodes]}")
    if not nodes:
        say("PROXY_SENSORS WARN LiDAR 렌더 노드를 못 찾음 — sim/run_proxy.sh check 로 LiDAR 토픽을 확인")


def set_view(stage):
    """뷰포트 카메라를 로봇 뒤 3인칭 카메라(ChaseCam)로 — 운전자용 화면, 기록에는 안 쓴다."""
    from pxr import Usd

    try:
        from omni.kit.viewport.utility import get_active_viewport

        cam = next((p for p in Usd.PrimRange(stage.GetPrimAtPath(ROBOT)) if p.GetName() == "ChaseCam"), None)
        vp = get_active_viewport()
        if vp is None or cam is None:
            say(f"PROXY_VIEW WARN viewport={vp is not None} chase_cam={cam is not None} — 기본 시점 그대로")
            return
        vp.camera_path = str(cam.GetPath())
        if VIEW_RES:
            vp.resolution = tuple(int(v) for v in VIEW_RES.lower().split("x"))
        say(f"PROXY_VIEW camera={cam.GetPath()} resolution={tuple(vp.resolution)}")
    except Exception as e:  # 뷰포트 API 가 달라도 주행에는 지장이 없으니 경고만
        say(f"PROXY_VIEW WARN {type(e).__name__}: {e}")


class RtfLog:
    """물리 스텝 시간의 합 ÷ 실제 시간을 RTF_EVERY 마다 찍는다. 타임라인 시각은 이 씬에서 늘지 않아 쓰지 않는다(9.30).
    Stop · Play 가 바뀌면 적고 다시 잰다."""

    def __init__(self, timeline):
        import omni.physx

        self.tl = timeline
        self.playing = timeline.is_playing()
        self.sim = 0.0
        self._sub = omni.physx.get_physx_interface().subscribe_physics_step_events(self._on_step)
        self.reset()

    def _on_step(self, dt):
        self.sim += dt

    def reset(self):
        self.w0, self.s0 = time.time(), self.sim

    def tick(self, *_):
        playing = self.tl.is_playing()
        if playing != self.playing:
            self.playing = playing
            say(f"PROXY_TIMELINE {'playing' if playing else 'stopped'}")
            self.reset()
            return
        w = time.time()
        if not playing:
            self.reset()
        elif w - self.w0 >= RTF_EVERY:
            ds = self.sim - self.s0
            say(f"PROXY_RTF {ds / (w - self.w0):.2f} (시뮬 {ds:.1f}s / 실제 {w - self.w0:.1f}s)")
            self.reset()


async def kit_main():
    import omni.kit.app
    import omni.timeline
    import omni.usd
    from isaacsim.core.utils.stage import is_stage_loading

    global _rtf_sub
    app = omni.kit.app.get_app()
    app.get_extension_manager().set_extension_enabled_immediate("isaacsim.ros2.bridge", True)
    for _ in range(60):  # 앱 UI 가 다 뜰 때까지(9.28 open_warmup 과 같음)
        await app.next_update_async()
    t0 = time.time()
    await omni.usd.get_context().open_stage_async(SCENE)
    while is_stage_loading():
        await app.next_update_async()
    for _ in range(120):  # 로딩이 끝나도 그래프가 준비되기까지 조금 더(9.28)
        await app.next_update_async()
    stage = omni.usd.get_context().get_stage()
    say(f"PROXY_LOADED {time.time() - t0:.1f}s {SCENE}")
    set_sensors(stage)
    set_view(stage)
    timeline = omni.timeline.get_timeline_interface()
    timeline.play()
    say(f"PROXY_PLAYING mode={MODE} domain={os.environ.get('ROS_DOMAIN_ID')} — 조작은 다른 터미널에서 sim/run_proxy.sh teleop")
    rtf = RtfLog(timeline)
    try:  # 끝나지 않는 코루틴 대신 갱신 이벤트에 붙인다 — 앱을 끌 때 대기 중인 작업이 남지 않게
        _rtf_sub = app.get_update_event_stream().create_subscription_to_pop(rtf.tick, name="fd_proxy_rtf")
    except Exception as e:
        say(f"PROXY_RTF WARN RTF 로그 없음 — {type(e).__name__}: {e}")


def headless_main():
    from isaacsim import SimulationApp

    app = SimulationApp({"headless": True, "active_gpu": 0, "physics_gpu": 0, "multi_gpu": False})

    import omni.timeline  # noqa: E402  (SimulationApp 뒤에 import 해야 함)
    import omni.usd  # noqa: E402
    from isaacsim.core.utils.extensions import enable_extension  # noqa: E402
    from isaacsim.core.utils.stage import is_stage_loading  # noqa: E402

    enable_extension("isaacsim.ros2.bridge")
    app.update()
    t0 = time.time()
    omni.usd.get_context().open_stage(SCENE)
    while is_stage_loading() or time.time() - t0 < 5:
        app.update()
    stage = omni.usd.get_context().get_stage()
    say(f"PROXY_LOADED {time.time() - t0:.1f}s {SCENE}")
    set_sensors(stage)
    timeline = omni.timeline.get_timeline_interface()
    timeline.play()
    duration = float(os.environ.get("FD_DURATION") or 0)
    say(f"PROXY_PLAYING mode=headless domain={os.environ.get('ROS_DOMAIN_ID')} "
        f"duration={f'{duration:.0f}s' if duration > 0 else 'Ctrl+C 까지'}")
    rtf = RtfLog(timeline)
    tp = time.time()
    try:
        while app.is_running() and (duration <= 0 or time.time() - tp < duration):
            app.update()
            rtf.tick()
    except KeyboardInterrupt:
        say("PROXY_STOPPING Ctrl+C")
    timeline.stop()
    app.update()
    say("PROXY_STOPPED")
    app.close()


_rtf_sub = None
if MODE == "headless":
    headless_main()
else:
    import asyncio

    asyncio.ensure_future(kit_main())
