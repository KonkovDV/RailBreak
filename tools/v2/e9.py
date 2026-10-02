from emu import *
tr2,srt=np.load('srt.npy').T
Mg=np.load('sensing_gnss_master_fix.npy'); Rv=np.load('sensing_gnss_rover_fix.npy')
r=pd.read_csv('repo/railbreak_backup_odometry/assets/ring.csv'); st=np.load('strue.npy')
vref=Rf[:,9]
for nm,G,nom in [('master',Mg,-9.873),('rover',Rv,2.563)]:
    m=(G[:,3]>=2)&(G[:,0]>100)
    sg=np.interp(G[m,1],r.s_m.values,st); tg=G[m,0]
    v=np.interp(tg,tr2,vref); 
    # model: sg - sref(tg) = off + v*tau  (ref lags => sref(tg) = strue(tg - tau) => sg - sref = nom + v*tau)
    d=sg-np.interp(tg,tr2,srt)
    A=np.c_[np.ones(m.sum()),v]; c,res,_,_=np.linalg.lstsq(A,d,rcond=None)
    mv=v>1
    print(nm,'n',m.sum(),'moving',mv.sum(),'fit off',c[0].round(3),'tau',c[1].round(3),'nominal',nom, 'resid std',np.std(d-A@c).round(3))
    for b in bursts(np.c_[tg,d,v]):
        print(f'   t {b[0,0]:6.0f} v {b[:,2].mean():5.2f} d {np.median(b[:,1]):+.2f}')
