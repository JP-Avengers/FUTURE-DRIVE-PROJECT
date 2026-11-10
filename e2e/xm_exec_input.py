"""직전 명령 입력을 실행 명령(exec_command)으로 — finetune.py --prev-action-source exec 가 씀 (E, 11.10)

X-Mobility 로더(model/dataset/isaac_sim_dataset.py)는 REQUIRED_COLUMNS 만 읽고 원소에 action(= driving_command, 교사 명령)만 넣는다.
여기서 하위 클래스로 exec_action(= exec_command, NaN 이면 교사 명령)을 더한다. 클래스를 이 파일(모듈)에 두는 이유: Windows 의
DataLoader 작업자는 spawn 이라 메인 프로세스에서 덧씌운 메서드가 전달되지 않음 — 하위 클래스는 피클 경로(xm_exec_input.…)로
작업자에서 다시 import 된다(spawn 은 부모 sys.path 를 넘김).
"""
import numpy as np

import model.dataset.isaac_sim_dataset as xds


class ExecIsaacSimDataset(xds.IsaacSimDataset):
    def _get_element(self, df, sample_index):
        el = super()._get_element(df, sample_index)
        ex = np.asarray(df.iloc[sample_index]["exec_command"], np.float64)
        el["exec_action"] = el["action"] if ex.shape != np.shape(el["action"]) or np.isnan(ex).any() else ex
        return el


def install():
    """데이터 모듈이 만드는 데이터셋을 위 클래스로(setup 이 모듈 전역 이름 IsaacSimDataset 을 찾음), 읽는 열에 exec_command."""
    if "exec_command" not in xds.REQUIRED_COLUMNS:
        xds.REQUIRED_COLUMNS = xds.REQUIRED_COLUMNS + ["exec_command"]
    xds.IsaacSimDataset = ExecIsaacSimDataset
