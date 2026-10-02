from geo import *
from pyproj import Geod
g=Geod(ellps='WGS84')
r=pd.read_csv('repo/railbreak_backup_odometry/assets/ring.csv')
r2d=180/np.pi
lat=LAT0+r.y_m/R0*r2d; lon=LON0+r.x_m/(R0*np.cos(LAT0/r2d))*r2d
_,_,dist=g.inv(lon.values[:-1],lat.values[:-1],lon.values[1:],lat.values[1:])
dl=np.hypot(np.diff(r.x_m),np.diff(r.y_m)); ds=np.diff(r.s_m)
print('sum ds',ds.sum(),'sum local',dl.sum(),'sum geod',dist.sum(),'ratio geod/s',dist.sum()/ds.sum())
hd=np.degrees(np.arctan2(np.diff(r.x_m),np.diff(r.y_m)))
for a in range(-180,180,45):
    m=(hd>=a)&(hd<a+45); 
    if m.sum(): print(f'heading {a:4d}..{a+45}: geod/s {dist[m].sum()/ds[m].sum():.5f} n={m.sum()}')
# true s
st=np.r_[0,np.cumsum(dist)]
np.save('strue.npy',st)
