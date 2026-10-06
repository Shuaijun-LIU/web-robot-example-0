#!/usr/bin/env python3
"""Offline CPU planning with live contact verification; exports actuator targets.

Only the separate IK MjData is pose-written. Live task bodies are never moved
by code after initialization, and export is conditional on all physical gates.
"""
import argparse
import json
from pathlib import Path
import mujoco
import numpy as np

HOME=np.array([1.570796,-.785398,0,-2.356194,0,1.570796,.785398])
ROOT=Path('public/assets/franka-cooperative').resolve()


def rotation_error(current,target):
    quat=np.empty(4);mujoco.mju_mat2Quat(quat,(target@current.T).ravel())
    error=np.empty(3);mujoco.mju_quat2Vel(error,quat,1.)
    return error


def top_rotation(yaw=0):
    c,s=np.cos(yaw),np.sin(yaw)
    return np.array([[c,s,0],[s,-c,0],[0,0,-1.]])


class Workcell:
    def __init__(self,scene):
        self.scene=scene;self.m=mujoco.MjModel.from_xml_path(str(ROOT/f'{scene}.xml'))
        self.d=mujoco.MjData(self.m);self.planner=mujoco.MjData(self.m)
        m,d=self.m,self.d
        self.joints=[[m.joint(f'r{a}_joint{j+1}').id for j in range(7)] for a in range(4)]
        self.qa=[m.jnt_qposadr[j] for j in self.joints];self.da=[m.jnt_dofadr[j] for j in self.joints]
        self.sites=[m.site(f'r{a}_tcp').id for a in range(4)]
        self.fingers=[{m.body(f'r{a}_{s}_finger').id for s in ['left','right']} for a in range(4)]
        self.robots={i for i in range(m.nbody) if m.body(i).name.startswith(tuple(f'r{a}_' for a in range(4)))}
        self.q=np.tile(HOME,(4,1));self.grip=np.full(4,255.);self.phases=[];self.holds={};self.history=[]
        self.max_forbidden=0.;self.max_grip=0.;self.loss={}
        for a in range(4):d.qpos[self.qa[a]]=HOME;d.ctrl[a*8:a*8+8]=[*HOME,255]
        mujoco.mj_forward(m,d)
        for _ in range(500):mujoco.mj_step(m,d)
        names=[o['name'] for o in json.loads((ROOT/'manifest.json').read_text())['scenes'][scene]['objects']]
        self.initial=[{'name':name,'position':d.body(name).xpos.tolist()} for name in names]

    def tcp(self,arm):return self.d.site_xpos[self.sites[arm]].copy()
    def rot(self,arm):return self.d.site_xmat[self.sites[arm]].reshape(3,3).copy()
    def pose(self,name):return self.d.body(name).xpos.copy(),self.d.body(name).xmat.reshape(3,3).copy()

    def solve(self,arm,position,rotation,seed):
        m,p=self.m,self.planner;p.qpos[:]=self.d.qpos;q=seed.copy()
        jp,jr=np.zeros((3,m.nv)),np.zeros((3,m.nv));site=self.sites[arm]
        for _ in range(350):
            p.qpos[self.qa[arm]]=q;mujoco.mj_kinematics(m,p);mujoco.mj_comPos(m,p)
            e=np.r_[position-p.site_xpos[site],.25*rotation_error(p.site_xmat[site].reshape(3,3),rotation)]
            if np.linalg.norm(e)<.00007:return q
            mujoco.mj_jacSite(m,p,jp,jr,site);J=np.vstack([jp[:,self.da[arm]],.25*jr[:,self.da[arm]]])
            delta=J.T@np.linalg.solve(J@J.T+1e-5*np.eye(6),e)
            q+=delta*min(1.,.1/max(np.linalg.norm(delta),1e-9))
            q=np.clip(q,m.jnt_range[self.joints[arm],0]+.005,m.jnt_range[self.joints[arm],1]-.005)
        raise RuntimeError(f'Arm {arm+1} IK {position.tolist()} residual {np.linalg.norm(e):.5f} q={q.tolist()}')

    def path(self,arm,target,rotation):
        m,p=self.m,self.planner;p.qpos[:]=self.d.qpos;p.qpos[self.qa[arm]]=self.q[arm];mujoco.mj_forward(m,p)
        origin=p.site_xpos[self.sites[arm]].copy();q0=np.empty(4);q1=np.empty(4)
        mujoco.mju_mat2Quat(q0,p.site_xmat[self.sites[arm]]);mujoco.mju_mat2Quat(q1,rotation.ravel())
        if q0@q1<0:q1=-q1
        path=[self.q[arm].copy()]
        for t in np.linspace(0,1,max(16,int(np.linalg.norm(target-origin)/.01)))[1:]:
            quat=(1-t)*q0+t*q1;quat/=np.linalg.norm(quat);matrix=np.empty(9);mujoco.mju_quat2Mat(matrix,quat)
            path.append(self.solve(arm,origin+(target-origin)*t,matrix.reshape(3,3),path[-1]))
        return np.array(path)

    def contacts(self):
        return [(self.m.geom_bodyid[c.geom1],self.m.geom_bodyid[c.geom2],c) for c in self.d.contact if c.dist<=0]

    def grasp(self,object,arm):
        id=self.m.body(object).id;touch={a if b==id else b for a,b,c in self.contacts() if id in [a,b]}
        aperture=sum(float(self.d.joint(f'r{arm}_finger_joint{j}').qpos[0]) for j in [1,2])
        return {'bilateral':self.fingers[arm]<=touch,'fingerContacts':len(self.fingers[arm]&touch),'aperture':aperture}

    def gate(self,g):
        m,d=self.m,self.d;kind=g['type']
        if kind=='home':
            error=max(float(np.max(np.abs(d.qpos[qa]-HOME))) for qa in self.qa)
            return error<.025,{'error':error}
        if kind=='grasp':
            o=self.grasp(g['object'],g['arm']);return o['bilateral'] and .002<o['aperture']<.081,o
        id=m.body(g['object']).id;touch={a if b==id else b for a,b,c in self.contacts() if id in [a,b]}
        if kind=='released':return not touch.intersection(set.union(*self.fingers)),{'fingerContacts':len(touch.intersection(set.union(*self.fingers)))}
        if kind=='height':return d.xpos[id,2]>=g['minZ'],{'height':float(d.xpos[id,2])}
        dof=m.jnt_dofadr[m.body_jntadr[id]];speed=float(np.linalg.norm(d.qvel[dof:dof+3]))
        if kind=='support':
            o={'supported':m.body(g['body']).id in touch,'speed':speed};return o['supported'] and speed<.03,o
        if kind=='inside':
            container=m.body(g['container']).id;relative=d.xmat[container].reshape(3,3).T@(d.xipos[id]-d.xpos[container])
            o={'relative':relative.tolist(),'supported':container in touch,'speed':speed}
            return o['supported'] and speed<.04 and np.linalg.norm(relative[:2])<g['radius'] and g['minZ']<=relative[2]<=g['maxZ'],o
        if kind=='scan':
            sp,sr=self.pose(g['scanner']);op,orr=self.pose(g['object']);axis=sr@np.array(g['axis'])
            delta=op+orr@np.array(g['target'])-(sp+sr@np.array(g['origin']));distance=float(delta@axis)
            o={'distance':distance,'offAxis':float(np.linalg.norm(delta-distance*axis)),'facing':float(-(orr@np.array(g['normal']))@axis)}
            return .025<=distance<=.09 and o['offAxis']<.02 and o['facing']>.9,o
        raise ValueError(g)

    def phase(self,name,duration,targets=None,grippers=None,touch=None,carry=None,gates=None,joint_targets=None,settle_timeout=0.):
        paths=[np.array([q.copy(),q.copy()]) for q in self.q]
        for a,(position,rotation) in (targets or {}).items():paths[a]=self.path(a,np.array(position),np.array(rotation))
        for a,q in (joint_targets or {}).items():paths[a]=np.array([self.q[a],q])
        end=self.grip.copy()
        for a,value in (grippers or {}).items():end[a]=value
        held=self.holds if carry is None else carry
        allowed=set()
        for a,object in {**self.holds,**(touch or {})}.items():
            for finger in self.fingers[a]:allowed.add(tuple(sorted((object,self.m.body(finger).name))))
        p={'name':name,'duration':duration,'paths':[path.tolist() for path in paths],
           'grippers':np.column_stack([self.grip,end]).tolist(),'allowedContacts':[list(pair) for pair in sorted(allowed)],
           'carry':[{'object':object,'arm':a} for a,object in held.items()],'gates':gates or []}
        if settle_timeout:p['settleTimeout']=settle_timeout
        self.phases.append(p);self.loss={}
        pairs={frozenset(self.m.body(n).id for n in pair) for pair in allowed}
        ticks=round(duration/.002)
        for tick in range(ticks):
            t=(tick+1)/ticks;s=t*t*t*(10+t*(-15+6*t))
            for a,path in enumerate(paths):
                u=s*(len(path)-1);i=min(len(path)-2,int(u));q=path[i]+(path[i+1]-path[i])*(u-i)
                self.d.ctrl[a*8:a*8+7]=q+self.d.qfrc_bias[self.da[a]]/self.m.actuator_gainprm[a*8:a*8+7,0]
                self.d.ctrl[a*8+7]=self.grip[a]+(end[a]-self.grip[a])*s
            mujoco.mj_step(self.m,self.d)
            for a,b,c in self.contacts():
                if a not in self.robots and b not in self.robots:continue
                good=frozenset([a,b]) in pairs
                if good:self.max_grip=max(self.max_grip,-float(c.dist))
                else:self.max_forbidden=max(self.max_forbidden,-float(c.dist))
                if c.dist<(-.0025 if good else -.001):
                    raise RuntimeError(f'{name}: collision {self.m.body(a).name}/{self.m.body(b).name} {c.dist:.6f} t={t:.3f}')
            for a,object in held.items():
                if self.grasp(object,a)['bilateral']:self.loss.pop(a,None)
                else:
                    self.loss.setdefault(a,self.d.time)
                    if self.d.time-self.loss[a]>.12:raise RuntimeError(f'{name}: lost grasp arm{a+1}/{object} {self.grasp(object,a)} tcp={self.tcp(a).tolist()} object={self.pose(object)[0].tolist()}')
        self.q=np.array([path[-1] for path in paths]);self.grip=end
        observations=[]
        for gate in p['gates']:
            passed,obs=self.gate(gate);observations.append(obs)
            if not passed:raise RuntimeError(f'{name}: gate {gate} failed {obs}')
        record={'phase':name,'time':float(self.d.time),'observations':observations,'tcp':[self.tcp(a).tolist() for a in range(4)],'objects':{o['name']:self.pose(o['name'])[0].tolist() for o in self.initial}}
        self.history.append(record);print(json.dumps(record),flush=True)

    def pick(self,arm,object,point,rotation,opening=255):
        above=np.array(point)+[0,0,.12]
        self.phase(f'Arm {arm+1} approach {object}',2.5,{arm:(above,rotation)},grippers={arm:opening},touch={arm:object})
        self.phase(f'Arm {arm+1} descend {object}',2,{arm:(point,rotation)},touch={arm:object})
        self.phase(f'Arm {arm+1} grip {object}',1.3,grippers={arm:0},touch={arm:object},gates=[{'type':'grasp','object':object,'arm':arm}])
        self.holds[arm]=object
        self.phase(f'Arm {arm+1} lift {object}',2,{arm:(above,rotation)},gates=[{'type':'grasp','object':object,'arm':arm}])

    def place(self,arm,object,xy,bottom,support,rotation=None):
        pos,_=self.pose(object);offset=self.tcp(arm)-pos;rotation=self.rot(arm) if rotation is None else rotation
        target=np.array([*xy,bottom])+offset
        self.phase(f'Arm {arm+1} transport {object}',3,{arm:(target+[0,0,.12],rotation)})
        self.phase(f'Arm {arm+1} lower {object}',2,{arm:(target,rotation)})
        support_gate={'type':'support','object':object,'body':support}
        self.phase(f'Settle {object} at support',.8)
        for _ in range(5):
            if self.gate(support_gate)[0]:break
            target[2]-=.002
            self.phase(f'Seek support for {object}',.6,{arm:(target,rotation)})
        self.phase(f'Confirm support for {object}',.3,gates=[support_gate])
        other={a:o for a,o in self.holds.items() if a!=arm}
        self.phase(f'Arm {arm+1} release {object}',1.2,grippers={arm:255},carry=other,gates=[{'type':'released','object':object},{'type':'support','object':object,'body':support}])
        del self.holds[arm]
        self.phase(f'Arm {arm+1} clear {object}',2,{arm:(target+[0,0,.12],rotation)},touch={arm:object})

    def move_object(self,name,arm,object,position,rotation,duration=3,gates=None):
        pos,rot=self.pose(object);local_point=rot.T@(self.tcp(arm)-pos);local_rot=rot.T@self.rot(arm)
        self.phase(name,duration,{arm:(np.array(position)+rotation@local_point,rotation@local_rot)},gates=gates)

    def export(self):
        plan={'version':1,'scene':self.scene,'initialJoints':HOME.tolist(),'initialObjects':self.initial,'phases':self.phases}
        (ROOT/f'{self.scene}-motion.json').write_text(json.dumps(plan,separators=(',',':'))+'\n')
        report={'success':True,'engine':mujoco.mj_versionString(),'maxForbiddenPenetration':self.max_forbidden,'maxGripPenetration':self.max_grip,'history':self.history}
        Path(f'artifacts/reports/cooperative-{self.scene}-native.json').write_text(json.dumps(report,indent=2)+'\n')


def scan(w):
    # Grasp annotation from RoboTwin scanner model_data0, transformed with the
    # same Y-up→Z-up conversion as the imported mesh.
    sp,sr=w.pose('scanner');scanner_grasp=sp+sr@np.array([0,.0115912,.048668])
    w.pick(1,'scanner',scanner_grasp,top_rotation(np.pi))
    # Source functional point 1 is the optical head center. Its local forward
    # direction becomes -Y after the common Y-up→Z-up asset conversion.
    body_rotation=np.array([[0,1,0],[-1,0,0],[0,0,1.]])
    for box,grasp_z,width,slot in [('tea_box',.045,.047,-.08),('coffee_box',.075,.036,.08)]:
        bp,_=w.pose(box)
        w.pick(0,box,bp+[0,0,grasp_z],top_rotation(np.pi))
        w.move_object(f'Present {box}',0,box,[0,-.10,.25],np.eye(3))
        gate={'type':'scan','object':box,'scanner':'scanner','origin':[0,-.0497656,.0500392],
              'axis':[0,-1,0],'target':[width,0,grasp_z-.01],'normal':[1,0,0]}
        bp,br=w.pose(box);target=bp+br@np.array(gate['target'])
        scanner_pos=target+[.055,0,0]-body_rotation@np.array(gate['origin'])
        w.move_object(f'Inspect {box}',1,'scanner',scanner_pos,body_rotation,gates=[gate])
        w.phase('Scanner clear of transfer lane',2,{1:([.335,-.16,.32],top_rotation(np.pi))})
        w.place(0,box,[0,0],.1205,'handoff')
        w.phase('Arm 1 clear of handoff',2.5,joint_targets={0:HOME})
        bp,_=w.pose(box)
        w.pick(2,box,bp+[0,0,grasp_z],top_rotation(0))
        tp,_=w.pose('order_tray')
        w.place(2,box,[tp[0]+slot,tp[1]],tp[2]+.009,'order_tray')
        w.phase('Arm 3 clear of packing area',2.5,joint_targets={2:HOME})
    w.place(1,'scanner',[.32,-.16],.11575,'scanner_dock')
    w.phase('Arm 2 return home',2.5,joint_targets={1:HOME})
    tp,_=w.pose('order_tray')
    tray_push_rotation=top_rotation(np.pi/2)
    point=tp+[0,.12,.016]
    w.phase('Arm 4 approach dispatch push',2.5,{3:(point+[0,0,.12],tray_push_rotation)},grippers={3:255},touch={3:'order_tray'})
    w.phase('Arm 4 lower behind tray',2,{3:(point,tray_push_rotation)},touch={3:'order_tray'})
    # Two separated fingertips push the outside wall; this is explicitly a
    # supported push, not a grasp or an airborne carry. No carry gate is used.
    end=point+np.array([-.4-tp[0],-.16-tp[1]-.025,0])
    w.phase('Arm 4 push loaded tray to dispatch',7,{3:(end,tray_push_rotation)},touch={3:'order_tray'},gates=[{'type':'support','object':'order_tray','body':'dispatch'}])
    w.phase('Arm 4 clear dispatched tray',2,{3:(end+[0,.04,.15],tray_push_rotation)},touch={3:'order_tray'},gates=[{'type':'released','object':'order_tray'}])
    w.phase('Order complete - all arms home',3,joint_targets={a:HOME for a in range(4)},gates=[
        {'type':'inside','object':box,'container':'order_tray','radius':.13,'minZ':.005,'maxZ':.085}
        for box in ['tea_box','coffee_box']]+[{'type':'home'},{'type':'support','object':'order_tray','body':'dispatch'}])


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--scene',choices=['scan','pot'],default='scan');args=parser.parse_args()
    work=Workcell(args.scene)
    try:
        if args.scene=='scan':scan(work)
        else:
            from cooperative_pot_task import run_pot
            run_pot(work)
        work.export()
        print('Complete physical task exported.',flush=True)
    except Exception as error:
        Path(f'artifacts/reports/cooperative-{args.scene}-native-failure.json').write_text(json.dumps({'success':False,'error':str(error),'history':work.history},indent=2)+'\n')
        raise


if __name__=='__main__':main()
