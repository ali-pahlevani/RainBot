#!/usr/bin/env python3
"""Generate maps/aws_hospital.{pgm,yaml} from aws_hospital.sdf."""
import glob
import math
import os
import urllib.parse
import xml.etree.ElementTree as ET

import numpy as np
from scipy import ndimage

from pickplace_arm_bringup.rack_table_layout import STATIC_TABLES, TABLE_SPAWN_Z

Z_LIDAR = 0.3143
RES = 0.05
BOUNDS = (-13.50, 13.50, -36.00, 22.00)
WALL_BOX = (-12.56, 12.56, -35.06, 21.06)
SEED_XY = (0.0, 7.75)          # a point known to be inside, in the lobby


def _rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = (math.cos(r), math.sin(r), math.cos(p),
                              math.sin(p), math.cos(y), math.sin(y))
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def _pose(text):
    v = [float(t) for t in (text or '0 0 0 0 0 0').split()]
    v += [0.0] * (6 - len(v))
    T = np.eye(4)
    T[:3, :3] = _rpy(*v[3:6])
    T[:3, 3] = v[:3]
    return T


def _load_obj(path):
    verts, tris = [], []
    with open(path, errors='replace') as fh:
        for line in fh:
            if line.startswith('v '):
                verts.append([float(t) for t in line.split()[1:4]])
            elif line.startswith('f '):
                idx = [int(t.split('/')[0]) for t in line.split()[1:]]
                idx = [i - 1 if i > 0 else len(verts) + i for i in idx]
                for k in range(1, len(idx) - 1):
                    tris.append([idx[0], idx[k], idx[k + 1]])
    if not verts or not tris:
        return np.zeros((0, 3, 3))
    V = np.array(verts)
    return V[np.array(tris)]


def _load_dae(path):
    r = ET.parse(path).getroot()
    ns = {'c': r.tag.split('}')[0].strip('{')}
    unit = r.find('.//c:unit', ns)
    scale = float(unit.get('meter', '1.0')) if unit is not None else 1.0
    up = (r.findtext('.//c:up_axis', namespaces=ns) or 'Y_UP').strip()

    geos = {}
    for g in r.findall('.//c:library_geometries/c:geometry', ns):
        m = g.find('c:mesh', ns)
        if m is None:
            continue
        srcs = {}
        for s in m.findall('c:source', ns):
            fa = s.find('c:float_array', ns)
            acc = s.find('.//c:accessor', ns)
            if fa is None or acc is None:
                continue
            st = int(acc.get('stride', '3'))
            srcs['#' + s.get('id')] = np.array(
                [float(t) for t in fa.text.split()]).reshape(-1, st)
        v = m.find('c:vertices', ns)
        if v is None:
            continue
        pid = [i.get('source') for i in v.findall('c:input', ns)
               if i.get('semantic') == 'POSITION']
        if not pid or pid[0] not in srcs:
            continue
        P = srcs[pid[0]][:, :3]
        vid = '#' + v.get('id')
        tris = []
        for prim in list(m):
            tag = prim.tag.split('}')[1]
            if tag not in ('polylist', 'triangles'):
                continue
            inputs = prim.findall('c:input', ns)
            off = [int(i.get('offset', 0)) for i in inputs
                   if i.get('source') == vid]
            if not off:
                continue
            stride = max(int(i.get('offset', 0)) for i in inputs) + 1
            pnode = prim.find('c:p', ns)
            if pnode is None:
                continue
            idx = np.array([int(t) for t in pnode.text.split()]
                           ).reshape(-1, stride)[:, off[0]]
            vc = prim.find('c:vcount', ns)
            if vc is None:
                for t in idx.reshape(-1, 3):
                    tris.append(t)
            else:
                k = 0
                for cnt in [int(t) for t in vc.text.split()]:
                    f = idx[k:k + cnt]
                    k += cnt
                    for j in range(1, cnt - 1):
                        tris.append([f[0], f[j], f[j + 1]])
        if tris:
            geos['#' + g.get('id')] = P[np.array(tris)]

    out = []
    for node in r.findall('.//c:visual_scene//c:node', ns):
        ig = node.find('c:instance_geometry', ns)
        if ig is None or ig.get('url') not in geos:
            continue
        M = np.eye(4)
        for ch in node:
            tag = ch.tag.split('}')[1]
            if tag == 'matrix':
                M = M @ np.array([float(t) for t in ch.text.split()]
                                 ).reshape(4, 4)
            elif tag == 'translate':
                T = np.eye(4)
                T[:3, 3] = [float(t) for t in ch.text.split()][:3]
                M = M @ T
            elif tag == 'rotate':
                v = [float(t) for t in ch.text.split()]
                ax = np.array(v[:3], float)
                n = np.linalg.norm(ax)
                if n > 0:
                    ax = ax / n
                    a = math.radians(v[3])
                    K = np.array([[0, -ax[2], ax[1]], [ax[2], 0, -ax[0]],
                                  [-ax[1], ax[0], 0]])
                    T = np.eye(4)
                    T[:3, :3] = (np.eye(3) + math.sin(a) * K
                                 + (1 - math.cos(a)) * K @ K)
                    M = M @ T
            elif tag == 'scale':
                T = np.eye(4)
                T[0, 0], T[1, 1], T[2, 2] = [float(t) for t in ch.text.split()][:3]
                M = M @ T
        T = geos[ig.get('url')]
        out.append((M[:3, :3] @ T.reshape(-1, 3).T).T.reshape(-1, 3, 3)
                   + M[:3, 3])
    if not out:
        out = list(geos.values())
    if not out:
        return np.zeros((0, 3, 3)), up
    return np.vstack(out) * scale, up


# Y_UP -> Z_UP, used only when the node transforms did not already do it.
_YUP = np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]], float)


def _mesh_triangles(path):
    """Triangles in the model's own Z-up frame, or None if unreadable."""
    ext = os.path.splitext(path)[1].lower()
    if ext == '.obj':
        return _load_obj(path)
    if ext in ('.dae', '.zae'):
        T, up = _load_dae(path)
        if len(T) == 0:
            return T
        cands = [T]
        if up.startswith('Y'):
            cands.append((_YUP @ T.reshape(-1, 3).T).T.reshape(-1, 3, 3))
        return min(cands, key=lambda C: abs(float(C[:, :, 2].min())))
    return None


def _resolve(uri, search):
    if uri.startswith('model://'):
        name = uri[len('model://'):]
    else:
        name = uri.rstrip('/').split('/models/')[-1].split('/')[0]
    for cand in (name, name.lower(), urllib.parse.unquote(name),
                 urllib.parse.quote(name).lower()):
        for base in search:
            d = os.path.join(base, cand)
            if os.path.isdir(d):
                sdf = sorted(glob.glob(d + '/**/model.sdf', recursive=True))
                if sdf:
                    return sdf[0]
    return None


def collision_triangles(sdf_path):
    """Every collision triangle of a model, in the model's own frame."""
    root = ET.parse(sdf_path).getroot()
    model = root.find('model')
    if model is None:
        return np.zeros((0, 3, 3))
    base = os.path.dirname(sdf_path)
    out = []
    Tm = _pose(model.findtext('pose'))
    for link in model.findall('link'):
        Tl = Tm @ _pose(link.findtext('pose'))
        for col in link.findall('collision'):
            Tc = Tl @ _pose(col.findtext('pose'))
            geom = col.find('geometry')
            if geom is None:
                continue
            tris = None
            mesh = geom.find('mesh')
            if mesh is not None:
                uri = (mesh.findtext('uri') or '').strip()
                rel = uri.split('/', 3)[-1] if uri.startswith('model://') else uri
                hits = glob.glob(os.path.join(base, '**', os.path.basename(rel)),
                                 recursive=True)
                if hits:
                    tris = _mesh_triangles(hits[0])
                    sc = mesh.findtext('scale')
                    if tris is not None and sc:
                        tris = tris * np.array([float(t) for t in sc.split()])
            box = geom.find('box')
            if box is not None:
                sx, sy, sz = [float(t) for t in box.findtext('size').split()]
                c = np.array([[-1, -1, -1], [1, -1, -1], [1, 1, -1], [-1, 1, -1],
                              [-1, -1, 1], [1, -1, 1], [1, 1, 1], [-1, 1, 1]],
                             float) * np.array([sx, sy, sz]) / 2.0
                f = [(0, 1, 2), (0, 2, 3), (4, 5, 6), (4, 6, 7), (0, 1, 5),
                     (0, 5, 4), (2, 3, 7), (2, 7, 6), (1, 2, 6), (1, 6, 5),
                     (0, 3, 7), (0, 7, 4)]
                tris = c[np.array(f)]
            cyl = geom.find('cylinder')
            if cyl is not None:
                rad = float(cyl.findtext('radius'))
                ln = float(cyl.findtext('length'))
                a = np.linspace(0, 2 * math.pi, 24, endpoint=False)
                ring = np.stack([rad * np.cos(a), rad * np.sin(a)], 1)
                tris = []
                for k in range(len(a)):
                    p0, p1 = ring[k], ring[(k + 1) % len(a)]
                    tris += [[[p0[0], p0[1], -ln / 2], [p1[0], p1[1], -ln / 2],
                              [p1[0], p1[1], ln / 2]],
                             [[p0[0], p0[1], -ln / 2], [p1[0], p1[1], ln / 2],
                              [p0[0], p0[1], ln / 2]]]
                tris = np.array(tris)
            if tris is None or len(tris) == 0:
                continue
            out.append((Tc[:3, :3] @ tris.reshape(-1, 3).T).T.reshape(-1, 3, 3)
                       + Tc[:3, 3])
    return np.vstack(out) if out else np.zeros((0, 3, 3))


def main():
    desc = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        '..', '..', 'pickplace_arm_description')
    desc = os.path.normpath(os.environ.get('DESC_SHARE', desc))
    search = [os.path.join(desc, 'models'),
              os.path.join(desc, 'aws_hospital_models'),
              os.path.join(desc, 'fuel_cache', 'fuel.gazebosim.org',
                           'openrobotics', 'models'),
              os.path.expanduser('~/.gz/fuel/fuel.gazebosim.org/'
                                 'openrobotics/models')]
    world = ET.parse(os.path.join(desc, 'worlds', 'aws_hospital.sdf'))
    w = world.getroot().find('world')

    X0, X1, Y0, Y1 = BOUNDS
    W = int(round((X1 - X0) / RES))
    H = int(round((Y1 - Y0) / RES))
    occ = np.zeros((H, W), bool)

    def seg(a, b):
        n = max(2, int(np.linalg.norm(b - a) / (RES * 0.4)) + 1)
        for s in np.linspace(0, 1, n):
            q = a + s * (b - a)
            j = int((q[0] - X0) / RES)
            i = int((q[1] - Y0) / RES)
            if 0 <= i < H and 0 <= j < W:
                occ[i, j] = True

    def stamp(tris):
        hits = 0
        for t in tris:
            z = t[:, 2]
            if z.min() > Z_LIDAR or z.max() < Z_LIDAR:
                continue
            pts = []
            for a, b in ((0, 1), (1, 2), (2, 0)):
                za, zb = t[a][2], t[b][2]
                if (za - Z_LIDAR) * (zb - Z_LIDAR) <= 0 and abs(zb - za) > 1e-12:
                    s = (Z_LIDAR - za) / (zb - za)
                    pts.append(t[a][:2] + s * (t[b][:2] - t[a][:2]))
            if len(pts) >= 2:
                seg(pts[0], pts[1])
                hits += 1
        return hits

    def stamp_footprint(tris):
        """Stamp a prop's whole footprint into the grid."""
        pts = tris.reshape(-1, 3)[:, :2]
        if len(pts) < 3:
            return
        try:
            from scipy.spatial import ConvexHull
            hull = pts[ConvexHull(pts).vertices]
        except Exception:
            return
        jj = ((hull[:, 0] - X0) / RES)
        ii = ((hull[:, 1] - Y0) / RES)
        i0, i1 = int(np.floor(ii.min())), int(np.ceil(ii.max()))
        j0, j1 = int(np.floor(jj.min())), int(np.ceil(jj.max()))
        i0, i1 = max(i0, 0), min(i1, H - 1)
        j0, j1 = max(j0, 0), min(j1, W - 1)
        if i1 < i0 or j1 < j0:
            return
        gi, gj = np.mgrid[i0:i1 + 1, j0:j1 + 1]
        inside = np.ones(gi.shape, bool)
        n = len(hull)
        for k in range(n):
            aj, ai = jj[k], ii[k]
            bj, bi = jj[(k + 1) % n], ii[(k + 1) % n]
            inside &= ((bj - aj) * (gi - ai) - (bi - ai) * (gj - aj)) >= -1e-9
        occ[i0:i1 + 1, j0:j1 + 1] |= inside

    cache, missing, empty = {}, [], []
    for inc in w.findall('include'):
        uri = (inc.findtext('uri') or '').strip()
        name = inc.findtext('name') or uri
        if uri not in cache:
            path = _resolve(uri, search)
            cache[uri] = collision_triangles(path) if path else None
            if cache[uri] is None:
                missing.append(uri)
        tris = cache[uri]
        if tris is None or len(tris) == 0:
            if uri not in missing:
                empty.append(name)
            continue
        T = _pose(inc.findtext('pose'))
        world_tris = (T[:3, :3] @ tris.reshape(-1, 3).T).T.reshape(-1, 3, 3) \
            + T[:3, 3]
        stamp(world_tris)

    for tx, ty, tyaw in STATIC_TABLES:
        path = _resolve('model://rack_table', search)
        if path is None:
            missing.append('rack_table')
            break
        tris = collision_triangles(path)
        if len(tris) == 0:
            empty.append('rack_table')
            break
        T = _pose('%f %f %f 0 0 %f' % (tx, ty, TABLE_SPAWN_Z, tyaw))
        stamp_footprint(
            (T[:3, :3] @ tris.reshape(-1, 3).T).T.reshape(-1, 3, 3) + T[:3, 3])

    bx0, bx1, by0, by1 = WALL_BOX
    for a, b in (((bx0, by0), (bx1, by0)), ((bx1, by0), (bx1, by1)),
                 ((bx1, by1), (bx0, by1)), ((bx0, by1), (bx0, by0))):
        seg(np.array(a, float), np.array(b, float))

    occ = ndimage.binary_closing(occ, np.ones((3, 3)))
    lab, _ = ndimage.label(~occ)
    si = int((SEED_XY[1] - Y0) / RES)
    sj = int((SEED_XY[0] - X0) / RES)
    free = lab == lab[si, sj]

    img = np.full((H, W), 205, np.uint8)
    img[free] = 254
    img[occ] = 0

    out_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                           '..', 'maps')
    out_dir = os.path.normpath(os.environ.get('MAP_DIR', out_dir))
    with open(os.path.join(out_dir, 'aws_hospital.pgm'), 'wb') as fh:
        fh.write(b'P5\n# generated by aws_hospital_map.py from aws_hospital.sdf'
                 b' - walls AND furniture, sliced at the LIDAR height\n')
        fh.write(b'%d %d\n255\n' % (W, H))
        fh.write(np.flipud(img).tobytes())
    with open(os.path.join(out_dir, 'aws_hospital.yaml'), 'w') as fh:
        fh.write('image: aws_hospital.pgm\nmode: trinary\n'
                 'resolution: %.3f\norigin: [%.2f, %.2f, 0]\n'
                 'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n'
                 % (RES, X0, Y0))

    print('free %.1f m2   occupied %.1f m2   unknown %.1f m2'
          % (free.sum() * RES * RES, occ.sum() * RES * RES,
             (img == 205).sum() * RES * RES))
    if missing:
        print('UNRESOLVED models (%d): %s' % (len(missing), missing[:5]))
    if empty:
        print('models with no collision at LIDAR height (%d): %s'
              % (len(empty), empty[:8]))


if __name__ == '__main__':
    main()
