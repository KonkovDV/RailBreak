from sync_sim import *
rear={round(s,6):(a,v) for a,s,v in B}
rows=np.array([(max(fa,rear[round(s,6)][0]),s,(fv+rear[round(s,6)][1])/7.2) for fa,s,fv in F if round(s,6) in rear])
def resample(rows,step=0.05,gapmax=0.35,mode='interp'):
    out=[]
    for i in range(len(rows)):
        a,s,v=rows[i]
        if i>0:
            pa,ps,pv=rows[i-1]; dt=s-ps
            if 0<dt<=gapmax:
                n=int(np.floor(dt/step-1e-9))
                for j in range(1,n+1):
                    tj=ps+j*step
                    if tj>=s-1e-6: break
                    vj=pv+(v-pv)*(tj-ps)/dt if mode=='interp' else pv
                    out.append((a,tj,vj))
        out.append((a,s,v))
    return np.array(out)
for step in [0.05,0.04,0.025]:
    R=resample(rows,step)
    span=R[-1,1]-R[0,1]; gaps=np.diff(R[:,1])
    print(f'step {step}: n {len(R)} rate {len(R)/span:.1f} Hz, max gap {gaps.max():.3f}', score(R[:,0]+0.001,R[:,1]+0.105,R[:,2]))
R=resample(rows,0.05,mode='hold'); print('hold',score(R[:,0]+0.001,R[:,1]+0.105,R[:,2]))
print('base',len(rows)/(rows[-1,1]-rows[0,1]),'Hz', 'gaps>0.1:',(np.diff(rows[:,1])>0.1001).mean())
# what do 1-second windows look like for rate
t=rows[:,1]; c=np.histogram(t,bins=np.arange(t[0],t[-1],1.0))[0]; print('1s windows with <10 msgs:',(c<10).mean())
