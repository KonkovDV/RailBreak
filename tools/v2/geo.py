import numpy as np, pandas as pd
from pyproj import Transformer
from scipy.spatial import cKDTree
R0=6378137.0; LAT0=55.804; LON0=37.425
TR=Transformer.from_crs("EPSG:4326","EPSG:32637",always_xy=True)
def ll2mgrs(lat,lon):
    e,n=TR.transform(lon,lat); return e-300000.0, n-6100000.0
def ring():
    r=pd.read_csv('repo/railbreak_backup_odometry/assets/ring.csv')
    r2d=180/np.pi
    lat=LAT0+r.y_m/R0*r2d; lon=LON0+r.x_m/(R0*np.cos(LAT0/r2d))*r2d
    X,Y=ll2mgrs(lat.values,lon.values)
    return r.s_m.values,X,Y,r.h_m.values
D=np.load('bag.npz')
