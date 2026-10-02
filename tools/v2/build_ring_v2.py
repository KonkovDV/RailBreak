#!/usr/bin/env python3
"""Rebuild ring.csv arc length and height profile (RailBreak assets v2).

1. s: WGS84 geodesic length of the polyline. The node's local frame
   (lat0, lon0, R = 6378137 m, equirectangular) shortens distances by
   0.06 % (N-S) .. 0.22 % (E-W), so the old s column was heading-dependent.
   x_m / y_m are kept: the node's latlon() still inverts them exactly.
2. h: robust profile. Median 31 m removes GNSS height steps (old grade up to 73 %),
   then Gaussian 8 m. grade = dh/ds on the new s, clipped to +-0.09.
3. stops.csv and off_ks_m are remapped to the new s; ring_len_m includes the closing segment.
usage: build_ring_v2.py <assets_in> <assets_out>
"""
import sys, re, shutil, numpy as np, pandas as pd
from pyproj import Geod
from scipy.signal import medfilt
from scipy.ndimage import gaussian_filter1d
src, dst = sys.argv[1], sys.argv[2]
shutil.copytree(src, dst, dirs_exist_ok=True)
meta = open(f'{src}/meta.yaml').read()
num = lambda k: float(re.search(rf'^{k}:\s*([-\d.eE+]+)', meta, re.M).group(1))
lat0, lon0, L_old = num('lat0_deg'), num('lon0_deg'), num('ring_len_m')
R0 = 6378137.0; r2d = 180 / np.pi
r = pd.read_csv(f'{src}/ring.csv')
lat = lat0 + r.y_m.values / R0 * r2d
lon = lon0 + r.x_m.values / (R0 * np.cos(lat0 / r2d)) * r2d
g = Geod(ellps='WGS84')
_, _, d = g.inv(lon[:-1], lat[:-1], lon[1:], lat[1:])
_, _, dc = g.inv(lon[-1], lat[-1], lon[0], lat[0])
s_new = np.r_[0.0, np.cumsum(d)] + 0.0
L_new = s_new[-1] + dc
old_s = r.s_m.values
old_close = L_old - old_s[-1]
def remap(s_old):
    s_old = np.mod(s_old, L_old)
    out = np.interp(s_old, old_s, s_new)
    tail = s_old > old_s[-1]
    out[tail] = s_new[-1] + (s_old[tail] - old_s[-1]) * dc / max(old_close, 1e-9)
    return out
h = r.h_m.values
n = len(h); pad = 40
hp = np.r_[h[-pad:], h, h[:pad]]
hm = medfilt(hp, 31)
hs = gaussian_filter1d(hm, 8.0)[pad:-pad]
grade = np.gradient(hs, s_new)
grade = np.clip(grade, -0.09, 0.09)
out = pd.DataFrame({'s_m': s_new, 'x_m': r.x_m, 'y_m': r.y_m, 'h_m': hs, 'grade': grade})
out.to_csv(f'{dst}/ring.csv', index=False, float_format='%.4f')
st = pd.read_csv(f'{src}/stops.csv')
st['s_m'] = remap(st.s_m.values)
st.to_csv(f'{dst}/stops.csv', index=False, float_format='%.3f')
off_new = float(remap(np.array([num('off_ks_m')]))[0])
meta2 = re.sub(r'^ring_len_m:.*$', f'ring_len_m: {L_new:.4f}', meta, flags=re.M)
meta2 = re.sub(r'^off_ks_m:.*$', f'off_ks_m: {off_new:.4f}', meta2, flags=re.M)
open(f'{dst}/meta.yaml', 'w').write(meta2)
print(f'ring_len {L_old:.3f} -> {L_new:.3f} (x{L_new/L_old:.5f}); max|grade| {np.abs(np.diff(h)/np.diff(old_s)).max():.3f} -> {np.abs(grade).max():.3f}; '
      f'max|h_old-h_new| {np.abs(h-hs).max():.2f} m; off_ks {num("off_ks_m")} -> {off_new:.3f}')
