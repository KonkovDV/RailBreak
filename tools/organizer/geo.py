"""Output frames of the node, mirrored for scoring (same series as track_odometer.hpp)."""

from __future__ import annotations

import numpy as np

A_WGS84 = 6378137.0
F_WGS84 = 1.0 / 298.257223563


def utm_forward(lat_deg, lon_deg, zone: int = 37):
    """WGS84 UTM easting/northing, Krueger series to n^6 (Karney 2011)."""
    n = F_WGS84 / (2.0 - F_WGS84)
    n2, n3, n4, n5, n6 = n**2, n**3, n**4, n**5, n**6
    A = A_WGS84 / (1.0 + n) * (1.0 + n2 / 4.0 + n4 / 64.0 + n6 / 256.0)
    al = [
        n / 2 - 2 * n2 / 3 + 5 * n3 / 16 + 41 * n4 / 180 - 127 * n5 / 288 + 7891 * n6 / 37800,
        13 * n2 / 48 - 3 * n3 / 5 + 557 * n4 / 1440 + 281 * n5 / 630 - 1983433 * n6 / 1935360,
        61 * n3 / 240 - 103 * n4 / 140 + 15061 * n5 / 26880 + 167603 * n6 / 181440,
        49561 * n4 / 161280 - 179 * n5 / 168 + 6601661 * n6 / 7257600,
        34729 * n5 / 80640 - 3418889 * n6 / 1995840,
        212378941 * n6 / 319334400,
    ]
    phi = np.radians(np.asarray(lat_deg, float))
    lam = np.radians(np.asarray(lon_deg, float) - (6.0 * zone - 183.0))
    c = 2.0 * np.sqrt(n) / (1.0 + n)
    t = np.sinh(np.arctanh(np.sin(phi)) - c * np.arctanh(c * np.sin(phi)))
    xi = np.arctan2(t, np.cos(lam))
    eta = np.arctanh(np.sin(lam) / np.sqrt(1.0 + t * t))
    se, sx = eta.copy(), xi.copy()
    for j, a in enumerate(al, start=1):
        se = se + a * np.cos(2 * j * xi) * np.sinh(2 * j * eta)
        sx = sx + a * np.sin(2 * j * xi) * np.cosh(2 * j * eta)
    return 500000.0 + 0.9996 * A * se, 0.9996 * A * sx


def ecef(lat_deg, lon_deg, h):
    e2 = F_WGS84 * (2.0 - F_WGS84)
    lat, lon = np.radians(lat_deg), np.radians(lon_deg)
    N = A_WGS84 / np.sqrt(1.0 - e2 * np.sin(lat) ** 2)
    return ((N + h) * np.cos(lat) * np.cos(lon), (N + h) * np.cos(lat) * np.sin(lon),
            (N * (1.0 - e2) + h) * np.sin(lat))


def mkrs_forward(lat_deg, lon_deg):
    """Moscow city grid angles (GKINP 01-268-02) on WGS84: meridian 37.5°, origin 55°40'."""
    lat0, lon0 = 55.0 + 40.0 / 60.0, 37.5
    e0, n0 = tmerc_wgs84(lat0, lon0, lon0, 1.0)
    e, n = tmerc_wgs84(lat_deg, lon_deg, lon0, 1.0)
    return e - e0, n - n0


def tmerc_wgs84(lat_deg, lon_deg, lon0_deg, k0):
    """WGS84 transverse Mercator, northing from the equator, Krueger series to n^6."""
    n = F_WGS84 / (2.0 - F_WGS84)
    n2, n3, n4, n5, n6 = n**2, n**3, n**4, n**5, n**6
    A = A_WGS84 / (1.0 + n) * (1.0 + n2 / 4.0 + n4 / 64.0 + n6 / 256.0)
    al = [
        n / 2 - 2 * n2 / 3 + 5 * n3 / 16 + 41 * n4 / 180 - 127 * n5 / 288 + 7891 * n6 / 37800,
        13 * n2 / 48 - 3 * n3 / 5 + 557 * n4 / 1440 + 281 * n5 / 630 - 1983433 * n6 / 1935360,
        61 * n3 / 240 - 103 * n4 / 140 + 15061 * n5 / 26880 + 167603 * n6 / 181440,
        49561 * n4 / 161280 - 179 * n5 / 168 + 6601661 * n6 / 7257600,
        34729 * n5 / 80640 - 3418889 * n6 / 1995840,
        212378941 * n6 / 319334400,
    ]
    phi = np.radians(np.asarray(lat_deg, float))
    lam = np.radians(np.asarray(lon_deg, float) - lon0_deg)
    c = 2.0 * np.sqrt(n) / (1.0 + n)
    t = np.sinh(np.arctanh(np.sin(phi)) - c * np.arctanh(c * np.sin(phi)))
    xi = np.arctan2(t, np.cos(lam))
    eta = np.arctanh(np.sin(lam) / np.sqrt(1.0 + t * t))
    se, sx = eta.copy(), xi.copy()
    for j, a in enumerate(al, start=1):
        se = se + a * np.cos(2 * j * xi) * np.sinh(2 * j * eta)
        sx = sx + a * np.sin(2 * j * xi) * np.cosh(2 * j * eta)
    return k0 * A * se, k0 * A * sx


def to_frame(lat, lon, h, start, mode: str = "mkrs_start", zone: int = 37,
             square=(400000.0, 6100000.0)):
    """Same axes as the node's output_frame. start = (lat0, lon0, h0). x is east, y is north."""
    lat, lon, h = (np.asarray(v, float) for v in (lat, lon, h))
    lat0, lon0, h0 = start
    if mode == "enu":
        X, Y, Z = ecef(lat, lon, h)
        X0, Y0, Z0 = ecef(lat0, lon0, h0)
        dx, dy, dz = X - X0, Y - Y0, Z - Z0
        sl, cl = np.sin(np.radians(lat0)), np.cos(np.radians(lat0))
        so, co = np.sin(np.radians(lon0)), np.cos(np.radians(lon0))
        return (-so * dx + co * dy, -sl * co * dx - sl * so * dy + cl * dz,
                cl * co * dx + cl * so * dy + sl * dz)
    if mode in ("mkrs", "mkrs_start"):
        e, n = mkrs_forward(lat, lon)
        if mode == "mkrs":
            return e, n, h
        e0, n0 = mkrs_forward(lat0, lon0)
        return e - e0, n - n0, h - h0
    e, n = utm_forward(lat, lon, zone)
    if mode == "mgrs":
        return e - square[0], n - square[1], h
    if mode == "grid_start":
        e0, n0 = utm_forward(lat0, lon0, zone)
        return e - e0, n - n0, h - h0
    raise ValueError(mode)
