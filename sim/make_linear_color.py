"""C 주차장 메쉬의 정점 색(사진에서 투영한 sRGB 값)을 선형으로 바꾼 덮어쓰기 레이어를 만든다.

Isaac Sim(RTX)은 primvars:displayColor 를 선형 색으로 읽는다. 사진값(sRGB)을 그대로 두면 아스팔트(sRGB 약 0.47)가
반사율 0.47 로 그려져 E2E 카메라 화면이 하얗게 날아간다(10.12 비교 렌더 — 변환하면 선형 약 0.19, Proxy 아스팔트 0.18 과 비슷).
메쉬 파일은 건드리지 않고 /World/Parking/Visual 의 primvars:displayColor 만 덮어쓴다. 루트 레이어(sim/scenes/parking/)가
C 의 parking_scene.usda 보다 앞에 subLayers 로 넣는다. C 가 선형 색으로 내보내게 되면 이 레이어는 뺀다.

Isaac Sim 을 띄우지 않고 pxr 만 쓴다(몇 초). 레포 루트(= $CD1)에서:
    envs/isaacsim_env/bin/python sim/make_linear_color.py data/usd/parking_v0.4b/usd/parking_mesh.usdc \
        data/usd/parking_v0.4b_dfix/color_linear.usdc
"""
import hashlib
import os
import sys

import numpy as np
from pxr import Sdf, Usd, UsdGeom, Vt

SRC, OUT = sys.argv[1], sys.argv[2]
TARGET = "/World/Parking/Visual"   # C 의 parking_scene.usda 가 메쉬 payload 를 /World/Parking 에 붙인다


def srgb_to_linear(c):
    return np.where(c <= 0.04045, c / 12.92, ((c + 0.055) / 1.055) ** 2.4).astype(np.float32)


src = Usd.Stage.Open(SRC)
vis = src.GetDefaultPrim().GetChild("Visual")
pv = UsdGeom.PrimvarsAPI(vis).GetPrimvar("displayColor")
srgb = np.array(pv.Get(), dtype=np.float32)
lin = srgb_to_linear(srgb)
sha = hashlib.sha256(open(SRC, "rb").read()).hexdigest()[:12]

if os.path.exists(OUT):
    sys.exit(f"이미 있음 — 덮어쓰지 않음: {OUT}")
os.makedirs(os.path.dirname(os.path.abspath(OUT)), exist_ok=True)
layer = Sdf.Layer.CreateNew(OUT)
layer.customLayerData = {
    "generated_by": "sim/make_linear_color.py",
    "source": os.path.relpath(SRC),
    "source_sha256_12": sha,
    "conversion": "sRGB -> linear (IEC 61966-2-1)",
}
stage = Usd.Stage.Open(layer)
over = stage.OverridePrim(TARGET)
UsdGeom.PrimvarsAPI(over).CreatePrimvar("displayColor", Sdf.ValueTypeNames.Color3fArray,
                                        pv.GetInterpolation()).Set(Vt.Vec3fArray.FromNumpy(lin))
layer.Save()
print(f"{OUT}: {len(lin)} 색 ({pv.GetInterpolation()}), 원본 sha {sha}, "
      f"평균 sRGB {srgb.mean():.3f} → 선형 {lin.mean():.3f}")
