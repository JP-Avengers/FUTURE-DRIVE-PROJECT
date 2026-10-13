"""하늘 돔 텍스처 — 지붕 키프레임의 하늘 화소를 높이각별로 평균한 latlong 그림 + 씬의 돔 조명에 얹는 덮어쓰기 레이어.

텍스처 없는 돔(단색)이면 로봇 카메라 화면 위쪽이 회색 한 가지 → 실제 영상의 하늘 그라데이션으로. 벽·물체가 아니라 조명 배경이다.
세기는 텍스처 위 반구 평균 밝기로 나눠 텍스처 없는 돔과 같은 밝기로 맞춘다.
"""
import os

import numpy as np
import open3d as o3d
from PIL import Image
from scipy import ndimage

from far_zone import sky_mask


def profile(sets, c):
    """sets = [(Frames, 씬 좌표 poses)] (지붕 카메라 — 위를 더 봄). 반환: (높이각 0~90° 1° 칸별 sRGB 색, 본 최대 높이각)."""
    acc, cnt = np.zeros((91, 3)), np.zeros(91)
    for fr, poses in sets:
        K = fr.intrinsic.intrinsic_matrix
        for i in range(0, len(poses), 3):
            img = np.asarray(o3d.io.read_image(fr.color[i]))
            d = np.asarray(o3d.io.read_image(fr.depth[i]))
            sky = sky_mask(img, c) & (d == 0)
            vv, uu = np.mgrid[0:img.shape[0]:4, 0:img.shape[1]:4]
            s = sky[vv, uu]
            ray = np.stack([(uu - K[0, 2]) / K[0, 0], (vv - K[1, 2]) / K[1, 1], np.ones(uu.shape)], -1) @ poses[i][:3, :3].T
            el = np.degrees(np.arctan2(ray[..., 2], np.hypot(ray[..., 0], ray[..., 1])))
            e = np.clip(el[s].astype(int), 0, 90)
            np.add.at(acc, e, img[vv, uu][s] / 255.0)
            np.add.at(cnt, e, 1)
    acc = ndimage.gaussian_filter1d(acc, 2.0, axis=0, mode="nearest")   # 2° 로 다듬기 (본 각도 끝에서 띠가 생기지 않게)
    cnt = ndimage.gaussian_filter1d(cnt, 2.0, mode="nearest")
    ok = cnt > 200
    if not ok.any():
        return np.tile([0.75, 0.82, 0.9], (91, 1)), 0
    prof = np.stack([np.interp(np.arange(91), np.nonzero(ok)[0], acc[ok, k] / cnt[ok]) for k in range(3)], -1)   # 본 각도 위는 같은 색
    return prof, int(np.nonzero(ok)[0].max())


def write(usd_dir, prof, ground_rgb, dome_intensity, W=2048):
    """textures/sky.png (latlong, 위 반구 = 높이각 그라데이션 · 아래 = 바닥색) + sky.usda (/World/Lights/Dome 덮어쓰기). 반환: (경로, 세기)."""
    from pxr import Sdf, Usd, UsdGeom
    H = W // 2
    el = 90.0 - (np.arange(H) + 0.5) * 180.0 / H
    rows = np.where(el[:, None] >= 0, prof[np.clip(el, 0, 90).astype(int)], np.asarray(ground_rgb)[None])
    os.makedirs(os.path.join(usd_dir, "textures"), exist_ok=True)
    Image.fromarray((np.clip(np.repeat(rows[:, None], W, 1), 0, 1) * 255).astype(np.uint8)).save(os.path.join(usd_dir, "textures", "sky.png"))
    lin = np.where(rows <= 0.04045, rows / 12.92, ((rows + 0.055) / 1.055) ** 2.4) @ [0.2126, 0.7152, 0.0722]
    w = np.cos(np.radians(el)) * (el > 0)                                 # 위 반구, 입체각 가중
    intensity = float(dome_intensity / max((lin * w).sum() / w.sum(), 1e-3))
    layer = Sdf.Layer.CreateAnonymous(".usda")
    st = Usd.Stage.Open(layer)
    UsdGeom.SetStageUpAxis(st, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(st, 1.0)
    st.OverridePrim("/World")
    st.OverridePrim("/World/Lights")
    dome = st.OverridePrim("/World/Lights/Dome")
    dome.CreateAttribute("inputs:texture:file", Sdf.ValueTypeNames.Asset).Set("./textures/sky.png")
    dome.CreateAttribute("inputs:texture:format", Sdf.ValueTypeNames.Token).Set("latlong")
    dome.CreateAttribute("poleAxis", Sdf.ValueTypeNames.Token).Set("Z")   # 텍스처 위 = +Z (UsdLux poleAxis — 모르는 버전이면 무시)
    dome.CreateAttribute("inputs:intensity", Sdf.ValueTypeNames.Float).Set(intensity)
    layer.customLayerData = dict(note=f"하늘 텍스처 — 텍스처 없는 돔 {dome_intensity} 과 같은 밝기. Z-up 에서 위아래가 뒤집혀 보이면 Dome 에 rotateX 90")
    path = os.path.join(usd_dir, "sky.usda")
    layer.Export(path)
    return path, intensity
