import subprocess
from hev import make
from hscore import evaluate
def go(name,a,st,args,lag,corr):
    make(a,st,'hev_t.csv',corr=corr)
    subprocess.run(['build/harness',a,'hev_t.csv','hout_t.csv']+args,check=True)
    rm,mx,zr,k=evaluate(a,'hout_t.csv',lag); print(f'{name:45s} RMSE {rm:.3f} max {mx:.2f} z {zr:.3f} k_end {k:.5f}')
if __name__=='__main__':
  a='av2_1.001234'
  for corr in [True,False]:
    for args in [[],['sigma_k0=0.0015'],['stop_sd_max=0'],['sigma_k0=0.0015','stop_sd_max=0'],['sigma_k0=0.0015','stop_sd_max=1.0'],['sigma_k0=0.0008','stop_sd_max=1.0']]:
        go(f'corr={corr} {args}',a,'both',args,0.05,corr)
  go('as-is assets, no-corr, stop off','repo/railbreak_backup_odometry/assets','master',['stop_sd_max=0'],0,False)
