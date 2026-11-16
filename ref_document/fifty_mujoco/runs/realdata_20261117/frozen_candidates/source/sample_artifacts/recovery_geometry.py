"""Mass-weighted whole-robot COM and wheel axle from the audited URDF.

All outputs are in the chassis frame. No added bodies, contacts, constraints or
torques. Reward geometry uses nominal CAD masses; perturbation evaluations must
use their actual modified model when assessing COM/support.
"""
import math
import xml.etree.ElementTree as ET


def rpy_matrix(r,p,y):
    cr,sr,cp,sp,cy,sy=math.cos(r),math.sin(r),math.cos(p),math.sin(p),math.cos(y),math.sin(y)
    return [[cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr],
            [sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr],[-sp,cp*sr,cp*cr]]


class RecoveryGeometry:
    def __init__(self,path,names,ops,device=None,dtype=None):
        self.ops=ops
        def array(value):
            if ops.__name__=='torch':return ops.tensor(value,device=device,dtype=dtype)
            return ops.asarray(value,dtype=dtype)
        self.array=array;tree=ET.parse(path)
        self.mass={};self.com={}
        for link in tree.findall('link'):
            name=link.get('name');inertial=link.find('inertial')
            self.mass[name]=float(inertial.find('mass').get('value'))
            self.com[name]=array([float(x) for x in inertial.find('origin').get('xyz').split()])
        self.total=sum(self.mass.values());self.joints=[];known={'base_link'}
        pending=list(tree.findall('joint'))
        while pending:
            ready=[j for j in pending if j.find('parent').get('link') in known]
            if not ready:raise ValueError('URDF tree is disconnected')
            for joint in ready:
                axis=[float(x) for x in joint.find('axis').get('xyz').split()]
                if axis!=[0.,0.,1.]:raise ValueError('Expected audited local Z-axis joints')
                origin=joint.find('origin');parent=joint.find('parent').get('link');child=joint.find('child').get('link')
                self.joints.append((parent,child,names.index(joint.get('name')),
                    array([float(x) for x in origin.get('xyz').split()]),
                    array(rpy_matrix(*[float(x) for x in origin.get('rpy','0 0 0').split()]))))
                known.add(child);pending.remove(joint)
        self.eye=array([[1.,0.,0.],[0.,1.,0.],[0.,0.,1.]])
        self.zero=array([0.,0.,0.]);self.forward=array([1.,0.,0.])

    def evaluate(self,q,gravity):
        ops=self.ops;zero=q[...,0]*0;one=zero+1
        stack=lambda values:ops.stack(values,dim=-1) if ops.__name__=='torch' else ops.stack(values,axis=-1)
        pos={'base_link':self.zero};rot={'base_link':self.eye}
        weighted=self.com['base_link']*self.mass['base_link']
        for parent,child,index,origin,fixed in self.joints:
            c,s=ops.cos(q[...,index]),ops.sin(q[...,index])
            local=stack([c,-s,zero,s,c,zero,zero,zero,one]).reshape(q.shape[:-1]+(3,3))
            pos[child]=pos[parent]+(rot[parent]@origin[...,None])[...,0]
            rot[child]=rot[parent]@fixed@local
            weighted=weighted+(pos[child]+(rot[child]@self.com[child][...,None])[...,0])*self.mass[child]
        com=weighted/self.total;axle=(pos['right_wheel']+pos['left_wheel'])/2
        up=-gravity;forward=self.forward-up[...,0:1]*up
        length=(forward*forward).sum(-1)**.5
        forward=forward/ops.clip(length,1e-8,None)[...,None]
        return dict(com=com,axle=axle,axle_com_error=((axle-com)*forward).sum(-1))
