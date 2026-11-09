"""Convierte las piezas STL del ensamblaje de SolidWorks (mm, Y arriba, frente hacia -X)
al marco del robot de ROS (m, X adelante, Y izquierda, Z arriba), origen entre las ruedas en el piso."""
import numpy as np, os, struct
D="C:/Users/jdvel/Documents/Universidad/7mo Semestre/6-Comunicaciones/Laboratorios/LAB_5/MiniSumo/Diseño_3D/MiniSumo - "
OUT=os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "ros2", "minisumo_ros", "meshes") + "/"
os.makedirs(OUT,exist_ok=True)
def load(n):
    b=open(D+n+'.STL','rb').read(); k=int.from_bytes(b[80:84],'little')
    a=np.frombuffer(b[84:84+k*50],dtype=np.dtype([('n','<f4',3),('v','<f4',(3,3)),('a','<u2')]))
    return a['v'].astype(np.float64)
# Ruedas: centro de cada una
WL=load('RUEDA_V1-4'); WR=load('Ensamblaje33-1 RUEDA_V1-1')
def center(v): m=v.reshape(-1,3); return (m.min(0)+m.max(0))/2
cl,cr=center(WL),center(WR)
X0=(cl[0]+cr[0])/2; Z0=(cl[2]+cr[2])/2; FLOOR=min(WL[...,1].min(),WR[...,1].min())
def to_robot(v, origin=(X0,FLOOR,Z0)):
    x,y,z=v[...,0]-origin[0],v[...,1]-origin[1],v[...,2]-origin[2]
    return np.stack([-x, z, y],-1)/1000.0      # adelante=-X, izquierda=+Z, arriba=+Y
def decimate(v, cell):
    """Simplificación por agrupamiento de vértices en una rejilla de 'cell' (m)."""
    q=np.round(v/cell).astype(np.int64)
    flat=q.reshape(-1,3)
    uniq,inv=np.unique(flat,axis=0,return_inverse=True)
    tri=inv.reshape(-1,3)
    ok=(tri[:,0]!=tri[:,1])&(tri[:,1]!=tri[:,2])&(tri[:,0]!=tri[:,2])
    tri=np.unique(np.sort(tri[ok],1),axis=0) if False else tri[ok]
    # posición: promedio de los vértices originales de cada celda
    acc=np.zeros((len(uniq),3)); cnt=np.zeros(len(uniq))
    np.add.at(acc,inv,flat*0+v.reshape(-1,3)); np.add.at(cnt,inv,1)
    pos=acc/cnt[:,None]
    return pos[tri]
def save(name, v):
    v=v.astype(np.float32); n=len(v)
    e1=v[:,1]-v[:,0]; e2=v[:,2]-v[:,0]; nr=np.cross(e1,e2); l=np.linalg.norm(nr,axis=1,keepdims=True); nr=nr/np.where(l==0,1,l)
    rec=np.zeros(n,dtype=np.dtype([('n','<f4',3),('v','<f4',(3,3)),('a','<u2')])); rec['n']=nr; rec['v']=v
    with open(OUT+name,'wb') as f:
        f.write(b'minisumo_ros: '+name.encode()+b' (m, X adelante, Z arriba)'.ljust(80-14-len(name),b' ')[:80-14-len(name)])
        f.write(struct.pack('<I',n)); f.write(rec.tobytes())
    m=v.reshape(-1,3); print(f'{name:16s} {n:7d} tri  {os.path.getsize(OUT+name)/1024:7.0f} kB  x {m[:,0].min()*1000:6.1f}..{m[:,0].max()*1000:6.1f}  y {m[:,1].min()*1000:6.1f}..{m[:,1].max()*1000:6.1f}  z {m[:,2].min()*1000:5.1f}..{m[:,2].max()*1000:5.1f} mm')
save('chasis.stl', to_robot(np.concatenate([load('Cuerpo-3'),load('tapa-1'),load('Soporte_PCB-1')])))
save('soportes.stl', to_robot(np.concatenate([load('Soporte_motor-1'),load('Soporte_motor-3')])))
mot=to_robot(np.concatenate([load('MOTOR-3'),load('Ensamblaje33-1 MOTOR-1')]))
save('motores.stl', decimate(mot, 0.0006))
# Ruedas centradas en su eje (el joint las gira alrededor de Y)
save('rueda_izq.stl', to_robot(WL, origin=cl))
save('rueda_der.stl', to_robot(WR, origin=cr))
print(f'radio rueda {(cl[1]-FLOOR)/1000:.4f} m | eje izq y={(cl[2]-Z0)/1000:+.4f} m, der y={(cr[2]-Z0)/1000:+.4f} m | separación {abs(cl[2]-cr[2]):.1f} mm')
