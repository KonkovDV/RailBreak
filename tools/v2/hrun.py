import subprocess, itertools
from hev import make
from hscore import evaluate
O='repo/railbreak_backup_odometry/assets'
cases=[('A as-is',O,'master',None,0.0,True),
       ('A no-corr',O,'master',None,0.0,False),
       ('V2 k=1.00123',"av2_1.001234",'master',None,0.0,True),
       ('V2 +both',"av2_1.001234",'both',None,0.0,True),
       ('V2 +both +lag',"av2_1.001234",'both',None,0.05,True),
       ('V2 +both +lag sk0=.0015',"av2_1.001234",'both','0.0015',0.05,True),
       ('V2 +both +lag sk0=.001',"av2_1.001234",'both','0.001',0.05,True),
       ('V2 k=1.00047 +both +lag',"av2_1.00047",'both',None,0.05,True),
       ('V2 k=1.00047 +both +lag sk0=.0015',"av2_1.00047",'both','0.0015',0.05,True),
       ('V2 no-corr both lag',"av2_1.001234",'both',None,0.05,False),
       ('V2 no-corr both lag sk0 .0015',"av2_1.001234",'both','0.0015',0.05,False),
       ]
for name,a,st,sk,lag,corr in cases:
    make(a,st,'hev_t.csv',corr=corr)
    cmd=['build/harness',a,'hev_t.csv','hout_t.csv']+([sk] if sk else [])
    subprocess.run(cmd,check=True)
    rm,mx,zr,k=evaluate(a,'hout_t.csv',lag)
    print(f'{name:40s} RMSE {rm:.3f} max {mx:.2f} z {zr:.3f} k_end {k:.5f}')
