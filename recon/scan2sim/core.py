"""단계 실행기와 공통 판정 형식.

단계마다 결과를 <out>/stages/<번호>_<이름>.json 한 파일로 남긴다 — 어느 파트의 코드든 이 형식만 맞추면 보고서에 들어간다.
    {"stage": "diagnose", "status": "pass|warn|fail|skip", "summary": "한 줄", "metrics": {...}, "messages": [...], "seconds": 1.2}
status: pass = 통과 · warn = 쓸 수 있지만 볼 것이 있음 · fail = 여기서 멈춤(이유는 messages) · skip = 이 실행에서는 안 함(이유는 summary)
"""
import json
import os
import time
import traceback

PASS, WARN, FAIL, SKIP = "pass", "warn", "fail", "skip"
MARK = {PASS: "통과", WARN: "주의", FAIL: "실패", SKIP: "건너뜀"}


class Result:
    def __init__(self, status=PASS, summary="", metrics=None, messages=None):
        self.status, self.summary, self.metrics, self.messages = status, summary, metrics or {}, messages or []

    def warn(self, msg):
        self.messages.append(msg)
        if self.status == PASS:
            self.status = WARN
        return self

    def fail(self, msg):
        self.messages.append(msg)
        self.status = FAIL
        return self


class Context(dict):
    """단계 사이에 넘기는 것: input · out · kind · profile · 산출물 경로들."""

    def path(self, *parts):
        p = os.path.join(self["out"], *parts)
        os.makedirs(os.path.dirname(p), exist_ok=True)
        return p


def run(stages, ctx, force=False):
    """stages = [(이름, 함수(ctx) → Result)]. 실패하면 멈춘다(force 면 계속). → [(이름, Result dict)]"""
    done = []
    os.makedirs(os.path.join(ctx["out"], "stages"), exist_ok=True)
    stopped = None
    for k, (name, fn) in enumerate(stages):
        t = time.time()
        if stopped and not (force or name == "report"):
            r = Result(SKIP, f"앞 단계({stopped})가 실패해서 하지 않음")
        else:
            try:
                r = fn(ctx)
            except SystemExit as e:                             # 파이프라인 함수가 문장과 함께 멈춘 것 (예: "바닥을 못 찾음") — 그 문장을 실패 이유로.
                # 잡지 않으면 단계 파일도 report.md 도 없이 엔진이 끝나 서버 · 화면이 이유를 알 수 없음 (CONTRACT 8-3 의 1번)
                why = e.code if isinstance(e.code, str) and e.code.strip() else f"멈춤 (종료 코드 {e.code})"
                r = Result(FAIL, why)
            except Exception as e:                              # 단계 안의 예외도 '실패 보고'로 남긴다
                r = Result(FAIL, f"예외: {type(e).__name__}: {e}", messages=traceback.format_exc().splitlines()[-6:])
        d = dict(stage=name, status=r.status, summary=r.summary, metrics=r.metrics, messages=r.messages, seconds=round(time.time() - t, 1))
        json.dump(d, open(os.path.join(ctx["out"], "stages", f"{k:02d}_{name}.json"), "w", encoding="utf-8"), ensure_ascii=False, indent=1, default=str)
        print(f"[{k + 1}/{len(stages)}] {name:9s} {MARK[r.status]:4s} {d['seconds']:6.1f}s  {r.summary}", flush=True)
        for m in r.messages[:6]:
            print(f"      - {m}", flush=True)
        done.append(d)
        if r.status == FAIL and not stopped:
            stopped = name
        ctx.setdefault("results", []).append(d)
    return done
