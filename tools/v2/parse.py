import sqlite3,struct,numpy as np
c=sqlite3.connect('bag/check-code/bags/30618_88aea4d9/30618_88aea4d9_0.db3')
def hdr(d):
    sec,nsec,L=struct.unpack_from('<iIi',d,4); o=16+L
    return sec+nsec*1e-9,o,d[16:16+L-1].decode()
def al(o,a): return o+((-(o-4))%a)
out={}
for tid,name in c.execute("select id,name from topics").fetchall():
    rows=c.execute("select timestamp,data from messages where topic_id=? order by timestamp",(tid,)).fetchall()
    recs=[]
    for ts,d in rows:
        t,o,fr=hdr(d)
        if 'bogie' in name:
            o=al(o,8); v=struct.unpack_from('<d',d,o)[0]; recs.append((ts*1e-9,t,v))
        elif 'driver' in name:
            v=struct.unpack_from('<b',d,o)[0]; recs.append((ts*1e-9,t,v))
        elif name.endswith('/vel'):
            o=al(o,8); vx,vy,vz=struct.unpack_from('<3d',d,o); recs.append((ts*1e-9,t,vx,vy,vz))
        elif name.endswith('/fix'):
            st=struct.unpack_from('<bH',d,o); o=al(o+1,2); o+=2; o=al(o,8)
            # status: int8 status, uint16 service
            lat,lon,alt=struct.unpack_from('<3d',d,o); o+=24; cov=struct.unpack_from('<9d',d,o); o+=72; ct=d[o]
            recs.append((ts*1e-9,t,lat,lon,alt,st[0],cov[0],cov[8]))
        elif 'kinematic' in name:
            L2=struct.unpack_from('<i',d,o)[0]; o=o+4+L2; o=al(o,8)
            px,py,pz,qx,qy,qz,qw=struct.unpack_from('<7d',d,o); o+=56+36*8
            vx,vy,vz,wx,wy,wz=struct.unpack_from('<6d',d,o)
            recs.append((ts*1e-9,t,px,py,pz,qx,qy,qz,qw,vx,vy,vz,wz))
    out[name.strip('/').replace('/','_')]=np.array(recs,dtype=float)
    print(name,len(recs),out[name.strip('/').replace('/','_')][:2])
np.savez('bag.npz',**out)
