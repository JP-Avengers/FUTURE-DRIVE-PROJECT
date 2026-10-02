"""여러 단계가 같이 쓰는 메쉬 도구: 지도 범위(footprint)와 그걸로 만드는 평면·벽, 구멍 메우기."""
import numpy as np
import open3d as o3d
from scipy import ndimage
from scipy.spatial import cKDTree


class Footprint:
    """메쉬(또는 xy 점들)가 덮는 범위를 위에서 본 cell 크기 격자.
    정점 사이 틈은 닫고(closing), 사방이 메쉬로 둘러싸인 빈 곳(방 안의 큰 바닥 구멍)은 안으로 채운다."""

    def __init__(self, mesh_or_xy, cell=0.1, margin=0.0):
        xy = np.asarray(mesh_or_xy.vertices)[:, :2] if hasattr(mesh_or_xy, "vertices") else np.asarray(mesh_or_xy)[:, :2]
        grow = int(np.ceil(margin / cell))
        self.cell, self.lo = cell, xy.min(0) - (grow + 2) * cell
        idx = self.index(xy)
        grid = np.zeros(idx.max(0) + grow + 3, bool)
        grid[idx[:, 0], idx[:, 1]] = True
        grid = ndimage.binary_fill_holes(ndimage.binary_closing(grid, iterations=2))
        self.grid = ndimage.binary_dilation(grid, iterations=grow) if grow else grid   # margin 만큼 바깥으로 넓힘

    def index(self, xy):
        return np.floor((xy - self.lo) / self.cell).astype(int)

    def inside(self, xy):
        idx = self.index(xy)
        ok = (idx >= 0).all(1) & (idx < self.grid.shape).all(1)
        res = np.zeros(len(xy), bool)
        res[ok] = self.grid[idx[ok, 0], idx[ok, 1]]
        return res

    def shrunk(self, margin):
        """margin 만큼 안쪽으로 줄인 사본 (경계에서 다른 면과 살짝 겹치게 할 때)."""
        import copy
        out = copy.copy(self)
        out.grid = ndimage.binary_erosion(self.grid, iterations=int(np.ceil(margin / self.cell)))
        return out

    def area(self):
        return float(self.grid.sum()) * self.cell ** 2

    def plane(self, z=0.0, exclude=None):
        """범위 안 칸을 덮는 높이 z 의 평면 (모서리 정점 공유, 법선 +Z). exclude(Footprint) 안의 칸은 뺀다."""
        ij = np.argwhere(self.grid)
        if exclude is not None:
            ij = ij[~exclude.inside(self.lo + (ij + 0.5) * self.cell)]
        if len(ij) == 0:
            return o3d.geometry.TriangleMesh()
        corners = np.stack([ij, ij + [1, 0], ij + [1, 1], ij + [0, 1]], axis=1)
        keys, inv = np.unique(corners.reshape(-1, 2), axis=0, return_inverse=True)
        q = inv.reshape(-1, 4)
        m = o3d.geometry.TriangleMesh(o3d.utility.Vector3dVector(np.c_[self.lo + keys * self.cell, np.full(len(keys), z)]),
                                      o3d.utility.Vector3iVector(np.vstack([q[:, [0, 1, 2]], q[:, [0, 2, 3]]])))
        m.compute_vertex_normals()
        return m

    def walls(self, height):
        """범위 테두리를 따라 z=0 부터 height 까지 세운 벽. 법선은 안쪽(범위 쪽)을 향한다."""
        g = np.pad(self.grid, 1)
        lo = self.lo - self.cell
        keys, tris = {}, []

        def vid(i, j, top):
            return keys.setdefault((i, j, top), len(keys))
        # (이웃 방향, 안쪽 칸 (i,j) 기준 경계 변의 두 모서리를 위에서 봐서 시계 방향으로) → 법선이 안쪽
        edges = {(1, 0): ((1, 1), (1, 0)), (-1, 0): ((0, 0), (0, 1)), (0, 1): ((0, 1), (1, 1)), (0, -1): ((1, 0), (0, 0))}
        for (di, dj), (a, b) in edges.items():
            outside = ~np.roll(g, (-di, -dj), axis=(0, 1))
            for i, j in np.argwhere(g & outside):
                pa, pb = (i + a[0], j + a[1]), (i + b[0], j + b[1])
                a0, b0, b1, a1 = vid(*pa, 0), vid(*pb, 0), vid(*pb, 1), vid(*pa, 1)
                tris += [[a0, b0, b1], [a0, b1, a1]]
        k = np.array(list(keys.keys()))
        pts = np.c_[lo + k[:, :2] * self.cell, k[:, 2] * height]
        m = o3d.geometry.TriangleMesh(o3d.utility.Vector3dVector(pts.astype(float)), o3d.utility.Vector3iVector(np.array(tris)))
        m.compute_vertex_normals()
        return m


def floor_colors(src_meshes, pts, band=0.05):
    """pts 위치에 가장 가까운 바닥(|z| < band, 위를 향한 면) 정점의 색. 바닥 정점이 없으면 회색."""
    xy, col = [], []
    for m in src_meshes:
        if m is None or not m.has_vertex_colors():
            continue
        if not m.has_vertex_normals():
            m.compute_vertex_normals()
        v, n = np.asarray(m.vertices), np.asarray(m.vertex_normals)
        sel = (np.abs(v[:, 2]) < band) & (n[:, 2] > 0.9)
        xy.append(v[sel, :2]); col.append(np.asarray(m.vertex_colors)[sel])
    if not xy or sum(len(x) for x in xy) == 0:
        return np.full((len(pts), 3), 0.6)
    _, j = cKDTree(np.vstack(xy)).query(np.asarray(pts)[:, :2])
    return np.vstack(col)[j]


def _nearest_colors(src, pts):
    """src 메쉬에서 가장 가까운 정점의 색."""
    if not src.has_vertex_colors():
        return None
    _, j = cKDTree(np.asarray(src.vertices)).query(pts)
    return np.asarray(src.vertex_colors)[j]


def fill_small_holes(mesh, hole_size):
    """테두리를 평평한 면으로 이어 작은 구멍을 메운다 (VTK fill holes, hole_size = 구멍을 감싸는 구의 반지름 m).
    Open3D 0.19 는 색·법선이 붙은 메쉬를 넣으면 조용히 종료되므로 모양만 넘기고 색은 다시 붙인다."""
    t = o3d.t.geometry.TriangleMesh()
    t.vertex.positions = o3d.core.Tensor(np.asarray(mesh.vertices), o3d.core.float32)
    t.triangle.indices = o3d.core.Tensor(np.asarray(mesh.triangles), o3d.core.int32)
    f = t.fill_holes(hole_size=hole_size)
    pos = f.vertex.positions.numpy().astype(np.float64)
    tri = f.triangle.indices.numpy().copy()

    # 새로 생긴 삼각형 중 주변 면과 반대 방향인 것은 뒤집기 (VTK 가 방향을 섞어서 만듦 → 광선 점검에서 '면 뒤쪽'으로 잡힘)
    if not mesh.has_vertex_normals():
        mesh.compute_vertex_normals()
    old_n = np.asarray(mesh.vertex_normals)
    new = tri[len(mesh.triangles):]
    if len(new) and len(pos) == len(old_n):
        face_n = np.cross(pos[new[:, 1]] - pos[new[:, 0]], pos[new[:, 2]] - pos[new[:, 0]])
        ref_n = old_n[new].sum(axis=1)                      # 세 꼭짓점의 원래 법선 합 = 주변 면 방향
        flip = np.sum(face_n * ref_n, axis=1) < 0
        new[flip] = new[flip][:, [0, 2, 1]]
        tri[len(mesh.triangles):] = new
    out = o3d.geometry.TriangleMesh(o3d.utility.Vector3dVector(pos), o3d.utility.Vector3iVector(tri))
    if mesh.has_vertex_colors():
        same = len(pos) == len(mesh.vertices) and np.allclose(pos, np.asarray(mesh.vertices), atol=1e-5)
        out.vertex_colors = mesh.vertex_colors if same else o3d.utility.Vector3dVector(_nearest_colors(mesh, pos))
    out.compute_vertex_normals()
    return out, len(out.triangles) - len(mesh.triangles)


def floor_patch(mesh, cell, band, below=0.002, footprint_cell=0.1):
    """지도 범위 안에서 바닥(|z| < band, 위를 향한 면)이 없는 cell 칸을 z=0 평면 조각으로 채운다.
    조각은 기존 바닥보다 below 만큼 낮게 둬서 겹치는 곳에서 깜빡임(z-fighting) 없이 기존 면이 보이게 한다.
    색은 가장 가까운 바닥 정점의 색. 반환: (조각 메쉬, 채운 칸 수)."""
    v = np.asarray(mesh.vertices)
    if not mesh.has_vertex_normals():
        mesh.compute_vertex_normals()
    n = np.asarray(mesh.vertex_normals)
    on_floor = (np.abs(v[:, 2]) < band) & (n[:, 2] > 0.9)
    fp = Footprint(mesh, footprint_cell)

    lo = v[:, :2].min(0) - cell
    shape = np.floor((v[:, :2].max(0) - lo) / cell).astype(int) + 2
    ii, jj = np.meshgrid(np.arange(shape[0]), np.arange(shape[1]), indexing="ij")
    centers = np.c_[lo[0] + (ii.ravel() + 0.5) * cell, lo[1] + (jj.ravel() + 0.5) * cell]

    # 칸이 기존 바닥 면으로 덮였나: 바닥 삼각형만 모아서, 칸 안 네 점에서 위→아래 광선. 하나라도 빠지면 채움
    # (정점 유무로 판단하면 의자 다리 옆 바닥 정점 몇 개 때문에 대부분 빈 칸을 건너뛰게 됨)
    tri = np.asarray(mesh.triangles)
    floor_tris = tri[on_floor[tri].all(axis=1)]
    rs = o3d.t.geometry.RaycastingScene()
    if len(floor_tris):
        rs.add_triangles(o3d.core.Tensor(v.astype(np.float32)), o3d.core.Tensor(floor_tris.astype(np.uint32)))
    covered = np.ones(len(centers), bool)
    for dx, dy in ((-.25, -.25), (.25, -.25), (-.25, .25), (.25, .25)):
        o = np.c_[centers + np.array([dx, dy]) * cell, np.full(len(centers), band + 0.05)]
        rays = np.c_[o, np.tile([0, 0, -1], (len(o), 1))].astype(np.float32)
        hit = np.isfinite(rs.cast_rays(o3d.core.Tensor(rays))["t_hit"].numpy()) if len(floor_tris) else np.zeros(len(o), bool)
        covered &= hit
    need = ~covered & fp.inside(centers)
    ci, cj = ii.ravel()[need], jj.ravel()[need]
    if len(ci) == 0:
        return o3d.geometry.TriangleMesh(), 0

    # 칸 모서리 정점을 공유해서 이어진 조각으로
    corners = np.stack([np.c_[ci, cj], np.c_[ci + 1, cj], np.c_[ci + 1, cj + 1], np.c_[ci, cj + 1]], axis=1)
    keys, inv = np.unique(corners.reshape(-1, 2), axis=0, return_inverse=True)
    q = inv.reshape(-1, 4)
    tris = np.vstack([q[:, [0, 1, 2]], q[:, [0, 2, 3]]])                  # 위에서 봐서 반시계 → 법선 +Z
    pts = np.c_[lo + keys * cell, np.full(len(keys), -below)]
    patch = o3d.geometry.TriangleMesh(o3d.utility.Vector3dVector(pts), o3d.utility.Vector3iVector(tris))
    if mesh.has_vertex_colors() and on_floor.any():
        _, j = cKDTree(v[on_floor, :2]).query(pts[:, :2])
        patch.vertex_colors = o3d.utility.Vector3dVector(np.asarray(mesh.vertex_colors)[on_floor][j])
    patch.compute_vertex_normals()
    return patch, len(ci)
