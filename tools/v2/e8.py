import numpy as np,pandas as pd
from scipy.signal import medfilt
r=pd.read_csv('repo/railbreak_backup_odometry/assets/ring.csv')
h=r.h_m.values; g=np.diff(h)/np.diff(r.s_m)
print('max |grade|',np.abs(g).max(),'n grade>0.08:',(np.abs(g)>0.08).sum())
bad=np.where(np.abs(g)>0.08)[0]; print(bad[:40], r.s_m.values[bad][:40])
hm=medfilt(h,31); dev=h-hm; print('max |h-median31|',np.abs(dev).max(),'n>0.5',(np.abs(dev)>0.5).sum())
i=np.argmax(np.abs(dev)); print('worst at s',r.s_m[i],h[i-5:i+6].round(2))
