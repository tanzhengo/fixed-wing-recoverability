#!/usr/bin/env python3
"""Independent collision-surface evaluator; never an input to the planner.

Snapshots use the static collision shapes, effective poses and effective scales
reported by Gazebo after loading the saved world. Meshes are exported by its own
MeshManager. Curved primitives are triangulated with a recorded conservative
surface-approximation correction. Distances refer to a point or position chord,
not the aircraft mesh. A returned cap is a lower bound, not an exact distance.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess

import numpy as np


def _sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _box_distance(low, high, other_low, other_high):
    delta = np.maximum(np.maximum(low-other_high, other_low-high), 0)
    return np.sqrt(np.sum(delta*delta, axis=-1))


def point_triangle_distances(point, triangles):
    """Exact Euclidean point-to-triangle distances, including degenerate faces."""
    point = np.asarray(point, dtype=float)
    t = np.asarray(triangles, dtype=float).reshape((-1, 3, 3))
    if not len(t):
        return np.empty(0)
    a, b, c = t[:, 0], t[:, 1], t[:, 2]
    ab, ac = b-a, c-a
    normal = np.cross(ab, ac); n2 = np.einsum('ij,ij->i', normal, normal)
    ap = point-a; numerator = np.einsum('ij,ij->i', ap, normal)
    projected = point - np.divide(numerator, n2, out=np.zeros_like(n2), where=n2>1e-24)[:, None]*normal
    v = projected-a
    d00 = np.einsum('ij,ij->i', ab, ab); d01 = np.einsum('ij,ij->i', ab, ac)
    d11 = np.einsum('ij,ij->i', ac, ac); d20 = np.einsum('ij,ij->i', v, ab); d21 = np.einsum('ij,ij->i', v, ac)
    denominator = d00*d11-d01*d01
    u = np.divide(d11*d20-d01*d21, denominator, out=np.zeros_like(n2), where=np.abs(denominator)>1e-24)
    w = np.divide(d00*d21-d01*d20, denominator, out=np.zeros_like(n2), where=np.abs(denominator)>1e-24)
    inside = (np.abs(denominator)>1e-24) & (u>=-1e-12) & (w>=-1e-12) & (u+w<=1+1e-12)
    best = np.where(inside, np.divide(numerator*numerator, n2, out=np.full_like(n2, np.inf), where=n2>1e-24), np.inf)
    for x, y in ((a,b),(b,c),(c,a)):
        edge = y-x; length = np.einsum('ij,ij->i', edge, edge)
        fraction = np.clip(np.divide(np.einsum('ij,ij->i', point-x, edge), length,
                                     out=np.zeros_like(length), where=length>1e-24), 0, 1)
        delta = point-(x+fraction[:, None]*edge)
        best = np.minimum(best, np.einsum('ij,ij->i', delta, delta))
    return np.sqrt(np.maximum(best, 0))


def _segment_edge_distances(p, q, x, y):
    u, v, w = q-p, y-x, p-x
    a = float(u@u); b = v@u; c = np.einsum('ij,ij->i', v, v)
    d = w@u; e = np.einsum('ij,ij->i', v, w)
    best = np.full(len(x), np.inf)
    def update(s, t):
        nonlocal best
        delta = w+np.asarray(s)[..., None]*u-np.asarray(t)[..., None]*v
        best = np.minimum(best, np.einsum('ij,ij->i', delta, delta))
    if a < 1e-24:
        fraction = np.clip(np.divide(e,c,out=np.zeros_like(c),where=c>1e-24),0,1)
        update(np.zeros(len(x)), fraction)
        return np.sqrt(np.maximum(best,0))
    determinant = a*c-b*b
    s = np.divide(b*e-c*d, determinant, out=np.zeros_like(c), where=determinant>1e-24)
    t = np.divide(a*e-b*d, determinant, out=np.zeros_like(c), where=determinant>1e-24)
    interior = (determinant>1e-24)&(s>=0)&(s<=1)&(t>=0)&(t<=1)
    delta = w+s[:, None]*u-t[:, None]*v
    best = np.where(interior,np.einsum('ij,ij->i',delta,delta),best)
    update(np.zeros(len(x)), np.clip(np.divide(e,c,out=np.zeros_like(c),where=c>1e-24),0,1))
    update(np.ones(len(x)), np.clip(np.divide(e+b,c,out=np.zeros_like(c),where=c>1e-24),0,1))
    update(np.clip(-d/a,0,1), np.zeros(len(x)))
    update(np.clip((b-d)/a,0,1), np.ones(len(x)))
    return np.sqrt(np.maximum(best,0))


def segment_triangle_distances(start, finish, triangles):
    """Exact closed-segment to triangle distances, including face crossings."""
    p, q = np.asarray(start,dtype=float), np.asarray(finish,dtype=float)
    t = np.asarray(triangles,dtype=float).reshape((-1,3,3))
    if not len(t):return np.empty(0)
    a,b,c=t[:,0],t[:,1],t[:,2];u=q-p
    best=np.minimum(point_triangle_distances(p,t),point_triangle_distances(q,t))
    ab,ac=b-a,c-a;normal=np.cross(ab,ac)
    n0=(p-a)*normal;n0=n0.sum(1);denominator=normal@u
    fraction=np.divide(-n0,denominator,out=np.full(len(t),-1.),where=np.abs(denominator)>1e-20)
    point=p+fraction[:,None]*u;v=point-a
    d00=(ab*ab).sum(1);d01=(ab*ac).sum(1);d11=(ac*ac).sum(1)
    d20=(v*ab).sum(1);d21=(v*ac).sum(1);den=d00*d11-d01*d01
    s=np.divide(d11*d20-d01*d21,den,out=np.full(len(t),-1.),where=np.abs(den)>1e-24)
    w=np.divide(d00*d21-d01*d20,den,out=np.full(len(t),-1.),where=np.abs(den)>1e-24)
    crosses=(fraction>=-1e-12)&(fraction<=1+1e-12)&(s>=-1e-12)&(w>=-1e-12)&(s+w<=1+1e-12)&(np.abs(denominator)>1e-20)
    best[crosses]=0
    for x,y in ((a,b),(b,c),(c,a)):
        best=np.minimum(best,_segment_edge_distances(p,q,x,y))
    return best


def _inside_closed_mesh(point, triangles):
    # Non-axis-aligned ray avoids the axis-aligned box diagonal degeneracies.
    direction=np.array([.812376123,.27184593,.515231789]);direction/=np.linalg.norm(direction)
    a,b,c=triangles[:,0],triangles[:,1],triangles[:,2]
    e1,e2=b-a,c-a;h=np.cross(np.broadcast_to(direction,e2.shape),e2)
    determinant=(e1*h).sum(1)
    inv=np.divide(1.,determinant,out=np.zeros_like(determinant),where=np.abs(determinant)>1e-12)
    s=point-a;u=inv*(s*h).sum(1);q=np.cross(s,e1);v=inv*(q@direction);distance=inv*(e2*q).sum(1)
    hits=(np.abs(determinant)>1e-12)&(u>=0)&(v>=0)&(u+v<=1)&(distance>1e-9)
    values=np.sort(distance[hits])
    # Shared face edges produce duplicate hits at exactly the same distance.
    return bool(len(values) and (1+np.count_nonzero(np.diff(values)>1e-7))%2)


class MeshTruth:
    def __init__(self,path):
        with np.load(path,allow_pickle=False) as data:
            self.triangles=np.asarray(data['triangles'],dtype=float)
            self.offsets=np.asarray(data['object_offsets'],dtype=np.int64)
            self.names=[str(x) for x in data['object_names']]
            self.closed=np.asarray(data['closed'],dtype=bool)
            self.errors=np.asarray(data['approximation_error'],dtype=float) if 'approximation_error' in data else np.zeros(len(self.names))
        if self.triangles.ndim!=3 or self.triangles.shape[1:]!=(3,3) or not np.isfinite(self.triangles).all():raise ValueError('Invalid mesh snapshot')
        if len(self.offsets)!=len(self.names)+1 or self.offsets[0]!=0 or self.offsets[-1]!=len(self.triangles) or np.any(np.diff(self.offsets)<=0):raise ValueError('Invalid mesh object ranges')
        self.tri_min=self.triangles.min(1);self.tri_max=self.triangles.max(1)
        self.minimum=np.array([self.tri_min[a:b].min(0) for a,b in zip(self.offsets[:-1],self.offsets[1:])])
        self.maximum=np.array([self.tri_max[a:b].max(0) for a,b in zip(self.offsets[:-1],self.offsets[1:])])
    def point_clearance(self,point,cap=30.):
        p=np.asarray(point,dtype=float)
        if p.shape!=(3,) or not np.isfinite(p).all() or not np.isfinite(cap) or cap<=0:raise ValueError('Invalid clearance query')
        best=float(cap);lower=_box_distance(self.minimum,self.maximum,p,p)-self.errors
        for i in np.argsort(lower):
            if lower[i]>best:break
            a,b=self.offsets[i:i+2];inside=(self.closed[i] and np.all(p>=self.minimum[i]) and np.all(p<=self.maximum[i]) and _inside_closed_mesh(p,self.triangles[a:b]))
            if inside:
                return -float(point_triangle_distances(p,self.triangles[a:b]).min())-self.errors[i]
            indices=np.flatnonzero(_box_distance(self.tri_min[a:b],self.tri_max[a:b],p,p)<=best+self.errors[i])+a
            if not len(indices):continue
            distance=float(point_triangle_distances(p,self.triangles[indices]).min())
            signed=distance-self.errors[i]
            best=min(best,signed)
            if best<0:return best
        return best
    def segment_clearance(self,start,finish,cap=30.):
        p,q=np.asarray(start,dtype=float),np.asarray(finish,dtype=float)
        if p.shape!=(3,) or q.shape!=(3,) or not np.isfinite(np.r_[p,q,cap]).all() or cap<=0:raise ValueError('Invalid clearance query')
        low,high=np.minimum(p,q),np.maximum(p,q);best=float(cap)
        lower=_box_distance(self.minimum,self.maximum,low,high)-self.errors
        for i in np.argsort(lower):
            if lower[i]>best:break
            a,b=self.offsets[i:i+2]
            if self.closed[i] and ((np.all(p>=self.minimum[i]) and np.all(p<=self.maximum[i]) and _inside_closed_mesh(p,self.triangles[a:b])) or (np.all(q>=self.minimum[i]) and np.all(q<=self.maximum[i]) and _inside_closed_mesh(q,self.triangles[a:b]))):return -1e-12
            indices=np.flatnonzero(_box_distance(self.tri_min[a:b],self.tri_max[a:b],low,high)<=best+self.errors[i])+a
            if len(indices):best=min(best,float(segment_triangle_distances(p,q,self.triangles[indices]).min())-self.errors[i])
            if best<=0:return best
        return best


def _rotation(quaternion):
    w,x,y,z=np.asarray(quaternion,dtype=float)/np.linalg.norm(quaternion)
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])


def _box_triangles(size):
    vertices=np.array([[-1,-1,-1],[1,-1,-1],[1,1,-1],[-1,1,-1],[-1,-1,1],[1,-1,1],[1,1,1],[-1,1,1]],float)*np.asarray(size)/2
    faces=np.array([[0,1,2],[0,2,3],[4,6,5],[4,7,6],[0,5,1],[0,4,5],[1,6,2],[1,5,6],[2,7,3],[2,6,7],[3,4,0],[3,7,4]])
    return vertices[faces]


def _is_closed(triangles):
    vertices=np.round(triangles.reshape((-1,3)),6)
    _,indices=np.unique(vertices,axis=0,return_inverse=True);indices=indices.reshape((-1,3))
    edges=np.sort(np.concatenate([indices[:,[0,1]],indices[:,[1,2]],indices[:,[2,0]]]),axis=1)
    _,counts=np.unique(edges,axis=0,return_counts=True)
    return bool(len(counts) and np.all(counts==2))


def _primitive(record):
    kind=record['type'];n=128
    if kind=='box':return _box_triangles(record['size']),0.,True
    if kind=='plane':
        normal=np.array(record['normal'],float);normal/=np.linalg.norm(normal)
        x=np.cross(normal,[1,0,0] if abs(normal[0])<.8 else [0,1,0]);x/=np.linalg.norm(x);y=np.cross(normal,x)
        a,b=np.asarray(record['size'])/2;v=np.array([-a*x-b*y,a*x-b*y,a*x+b*y,-a*x+b*y])
        return v[[[0,1,2],[0,2,3]]],0.,False
    if kind=='cylinder':
        r,h=record['radius'],record['length']/2;theta=np.arange(n)*2*np.pi/n
        bottom=np.c_[r*np.cos(theta),r*np.sin(theta),np.full(n,-h)];top=bottom.copy();top[:,2]=h
        triangles=[]
        for i in range(n):
            j=(i+1)%n;triangles.extend([[bottom[i],bottom[j],top[j]],[bottom[i],top[j],top[i]],[[0,0,-h],bottom[j],bottom[i]],[[0,0,h],top[i],top[j]]])
        return np.array(triangles),r*(1-np.cos(np.pi/n)),True
    if kind=='sphere':
        r=record['radius'];latitude=np.linspace(-np.pi/2,np.pi/2,65);longitude=np.arange(n)*2*np.pi/n
        vertices=np.array([[[r*np.cos(t)*np.cos(p),r*np.cos(t)*np.sin(p),r*np.sin(t)] for p in longitude] for t in latitude]);triangles=[]
        for i in range(64):
            for j in range(n):
                k=(j+1)%n
                triangles.extend([[vertices[i,j],vertices[i,k],vertices[i+1,k]],[vertices[i,j],vertices[i+1,k],vertices[i+1,j]]])
        return np.array(triangles),r*(1-np.cos(np.sqrt((np.pi/64)**2+(2*np.pi/n)**2)/2)),True
    raise ValueError('Unsupported collision geometry: '+kind)


def _resolve_uri(uri,roots):
    if uri.startswith('file://'):path=Path(uri[7:])
    elif uri.startswith('model://'):
        suffix=uri[8:];path=next((r/suffix for r in roots if (r/suffix).exists()),None)
        if path is None:raise FileNotFoundError(uri)
    else:path=Path(uri)
    if not path.is_file():raise FileNotFoundError(path)
    return path.resolve()


def export_snapshot(metadata,mesh_export,output,world=None):
    records=json.loads(Path(metadata).read_text())['collisions']
    roots=[Path(x) for x in os.environ.get('GAZEBO_MODEL_PATH','').split(':') if x]+[Path.home()/'.gazebo/models']
    roots += [Path('/usr/share/gazebo-11/models')]
    mesh_cache={};all_triangles=[];offsets=[0];names=[];closed=[];errors=[];assets={};objects=[]
    for record in records:
        if record['type']=='mesh':
            path=_resolve_uri(record['uri'],roots);assets[str(path)]=_sha(path)
            if str(path) not in mesh_cache:
                mesh_cache[str(path)]=json.loads(subprocess.check_output([str(mesh_export),str(path)],text=True))
            triangles=[]
            for sub in mesh_cache[str(path)]['submeshes']:
                # Gazebo collision meshes use triangle faces. Collada files
                # may also carry decorative POINTS/LINES/LINESTRIPS.
                primitive=int(sub.get('primitive_type',3))
                if primitive in (0,1,2):continue
                if primitive!=3 or len(sub['indices'])%3:
                    raise ValueError('Unsupported non-triangle collision primitive in '+str(path))
                points=np.array(sub['vertices'],float)*record['scale'];triangles.extend(points[np.array(sub['indices']).reshape((-1,3))])
            triangles=np.array(triangles);error=0.;is_closed=_is_closed(triangles)
        else:triangles,error,is_closed=_primitive(record)
        triangles=triangles@_rotation(record['quaternion']).T+record['position']
        normals=np.cross(triangles[:,1]-triangles[:,0],triangles[:,2]-triangles[:,0]);triangles=triangles[(normals*normals).sum(1)>1e-20]
        if not len(triangles):raise ValueError('Empty static collision: '+record['name'])
        all_triangles.append(triangles);offsets.append(offsets[-1]+len(triangles));names.append(record['name']);closed.append(is_closed);errors.append(error)
        objects.append({'name':record['name'],'kind':record['type'],'triangles':len(triangles),'closed':is_closed,'approximation_error':error})
    output=Path(output);output.parent.mkdir(parents=True,exist_ok=True)
    np.savez_compressed(output,triangles=np.concatenate(all_triangles),object_offsets=np.array(offsets,np.int64),object_names=np.array(names),closed=np.array(closed,bool),approximation_error=np.array(errors))
    manifest={'schema':'static_collision_triangles_v1','source_world':str(Path(world).resolve()) if world else None,'world_sha256':_sha(world) if world else None,'metadata_sha256':_sha(metadata),'mesh_export_sha256':_sha(mesh_export),'mesh_assets_sha256':assets,'snapshot_sha256':_sha(output),'objects':objects,'object_count':len(names),'triangle_count':offsets[-1],'distance_semantics':'Physical collision surface. Closed solids also reject interior points; curved primitive triangulations subtract their surface approximation bound. Caps are lower bounds. No planner input.'}
    output.with_suffix('.manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    return manifest


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--metadata',type=Path,required=True);p.add_argument('--mesh-export',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--world',type=Path);args=p.parse_args()
    m=export_snapshot(args.metadata,args.mesh_export,args.output,args.world);print(json.dumps({k:m[k] for k in ['object_count','triangle_count','snapshot_sha256']},indent=2))


if __name__=='__main__':main()
