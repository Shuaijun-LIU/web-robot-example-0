#!/usr/bin/env python3
"""CPU-only four-arm relay. Live bodies move only through finite actuators/contact."""
import importlib.util
import json
from pathlib import Path
import numpy as np
import mujoco

spec=importlib.util.spec_from_file_location('cooperative_solver',Path(__file__).with_name('solve-cooperative-tasks.py'))
base=importlib.util.module_from_spec(spec);spec.loader.exec_module(base)
ROOT,HOME=base.ROOT,base.HOME
UPRIGHT={'type':'upright','object':'panel','axis':[0,0,1],'minAlignment':float(np.cos(np.pi/18)),'maxSpeed':.5}
ROTATIONS=[np.array(r,dtype=float) for r in [
    [[-1,0,0],[0,0,1],[0,1,0]], [[0,0,-1],[-1,0,0],[0,1,0]],
    [[1,0,0],[0,0,-1],[0,1,0]], [[0,0,1],[1,0,0],[0,1,0]]]]
# Inclined edge entry keeps the wrist away from the folded-arm singularity.
# Keep the finger roots outside the edge while the pads straddle both faces.
ANGLE=np.deg2rad(35);TILT=np.array([[1,0,0],[0,np.cos(ANGLE),-np.sin(ANGLE)],[0,np.sin(ANGLE),np.cos(ANGLE)]])
ROTATIONS=[r@TILT for r in ROTATIONS]
# The east underside has a decorative groove outside x=.180. An extra
# 10-mm inset puts Arm 2's lower pad on the REAL flat face, not a hull bridge.
GRIPS=[np.array(p) for p in [[0,-.16,.013],[.16,0,.013],[0,.16,.013],[-.17,0,.013]]]


def grasp(arm):return {'type':'grasp','object':'panel','arm':arm}


class RelayWorkcell(base.Workcell):
    def __init__(self):
        super().__init__('relay');self.max_limited=0.;self.qualify=False;self.last_sample=-1;self.witness={}
        self.panel_geom=self.m.geom('panel_contact_0').id
        obstacles={self.m.body(f'barrier_{i}').id for i in range(3)}
        self.obstacle_bounds=[self.mesh_bounds(g) for g in range(self.m.ngeom) if self.m.geom_bodyid[g] in obstacles and self.m.geom_contype[g]]
        self.evidence={'airborneSamples':0,'minBilateralSupports':4,'minSurfaceSupports':4,'surfaceFallbackSamples':0,
          'maxSurfaceGap':0.,'maxTilt':0.,'minAirborneHeight':1.,'minObstacleClearance':1.}
        for o in self.initial:o.update(quaternion=self.d.body(o['name']).xquat.tolist(),orientationTolerance=.12)

    def mesh_bounds(self,g):
        m,d=self.m,self.d;mesh=m.geom_dataid[g];v=m.mesh_vert[m.mesh_vertadr[mesh]:m.mesh_vertadr[mesh]+m.mesh_vertnum[mesh]]
        world=v@d.geom_xmat[g].reshape(3,3).T+d.geom_xpos[g]
        return np.array([world.min(axis=0),world.max(axis=0)])

    def witness_gap(self,finger):
        if finger not in self.witness:return float('inf')
        t,op,fp=self.witness[finger]
        if not 0<=self.d.time-t<=.0060001:return float('inf')
        p,r=self.pose('panel');bp=self.d.xpos[finger];br=self.d.xmat[finger].reshape(3,3)
        return float(np.linalg.norm((p+r@op)-(bp+br@fp)))

    def contacts(self):
        contacts=super().contacts();id=self.m.body('panel').id
        obstacles={self.m.body(f'barrier_{i}').id for i in range(3)}
        for a,b,c in contacts:
            if id in [a,b] and (a in obstacles or b in obstacles):
                self.max_limited=max(self.max_limited,-float(c.dist))
                if c.dist<-.001:raise RuntimeError(f'Panel/barrier penetration {-c.dist:.6f} m')
        latest={}
        for a,b,c in contacts:
            if id not in [a,b]:continue
            finger=b if a==id else a
            if finger not in set.union(*self.fingers):continue
            if finger in latest and latest[finger][0]<-c.dist:continue
            p1=c.pos-c.frame[:3]*c.dist/2;p2=c.pos+c.frame[:3]*c.dist/2
            op=self.d.xmat[id].reshape(3,3).T@((p1 if a==id else p2)-self.d.xpos[id])
            fp=self.d.xmat[finger].reshape(3,3).T@((p1 if a==finger else p2)-self.d.xpos[finger])
            latest[finger]=(-c.dist,op,fp)
        for finger,(_,op,fp) in latest.items():self.witness[finger]=(self.d.time,op,fp)
        if self.qualify and self.d.time>self.last_sample:
            self.last_sample=self.d.time;tilt=float(np.arccos(np.clip(self.pose('panel')[1][2,2],-1,1)))
            self.evidence['maxTilt']=max(self.evidence['maxTilt'],tilt)
            if tilt>np.pi/18:raise RuntimeError(f'Panel tilt {np.degrees(tilt):.3f} deg')
            panel_bounds=self.mesh_bounds(self.panel_geom)
            gap=min(max(0,float(np.max(panel_bounds[0]-b[1])),float(np.max(b[0]-panel_bounds[1]))) for b in self.obstacle_bounds)
            self.evidence['minObstacleClearance']=min(self.evidence['minObstacleClearance'],gap)
            if gap<=.002:raise RuntimeError(f'Panel/obstacle separating-plane clearance {gap:.6f} m')
            touching={a if b==id else b for a,b,c in contacts if id in [a,b]}
            supported=any(self.m.body(n).id in touching for n in ['panel_start','panel_output'])
            if not supported:
                count=sum(f<=touching for f in self.fingers)
                self.evidence['airborneSamples']+=1
                self.evidence['minAirborneHeight']=min(self.evidence['minAirborneHeight'],float(self.pose('panel')[0][2]))
                self.evidence['minBilateralSupports']=min(self.evidence['minBilateralSupports'],count)
                surface_count=count
                for arm in self.holds:
                    if self.fingers[arm]<=touching or self.d.ctrl[arm*8+7]>1:continue
                    # Follow actual material surface points from the previous
                    # contact, at most 6 ms old. Do not trust mesh-distance zero.
                    gap=max(0 if f in touching else self.witness_gap(f) for f in self.fingers[arm])
                    if gap<=.00005:
                        surface_count+=1;self.evidence['surfaceFallbackSamples']+=1
                        self.evidence['maxSurfaceGap']=max(self.evidence['maxSurfaceGap'],gap)
                self.evidence['minSurfaceSupports']=min(self.evidence['minSurfaceSupports'],surface_count)
                if surface_count<2:raise RuntimeError(f'Only {surface_count} surface-qualified supports at {self.d.time:.3f}, touching={sorted(touching)}')
        return contacts

    def gate(self,g):
        if g['type']!='upright':return super().gate(g)
        id=self.m.body(g['object']).id;dof=self.m.jnt_dofadr[self.m.body_jntadr[id]]
        o={'alignment':float((self.pose(g['object'])[1]@np.array(g['axis']))[2]),'speed':float(np.linalg.norm(self.d.qvel[dof:dof+3]))}
        return o['alignment']>=g['minAlignment'] and o['speed']<=g['maxSpeed'],o

    def coupled(self,name,position,duration=3):
        pos,rot=self.pose('panel');targets={}
        for arm in self.holds:
            # All TCP goals come from the same observed rigid-body transform.
            local=rot.T@(self.tcp(arm)-pos);local_rotation=rot.T@self.rot(arm)
            targets[arm]=(np.array(position)+local,local_rotation)
        self.phase(name,duration,targets,gates=[UPRIGHT]+[grasp(a) for a in self.holds])

    def export(self):
        plan={'version':1,'scene':'relay','initialJoints':HOME.tolist(),'initialObjects':self.initial,
          'contactLimits':[{'a':'panel','b':f'barrier_{i}','maxPenetration':.001} for i in range(3)],'phases':self.phases}
        (ROOT/'relay-motion.json').write_text(json.dumps(plan,separators=(',',':'))+'\n')
        report={'success':True,'engine':mujoco.mj_versionString(),'maxForbiddenPenetration':self.max_forbidden,
          'maxGripPenetration':self.max_grip,'maxLimitedContactPenetration':self.max_limited,'evidence':self.evidence,'history':self.history}
        Path('artifacts/reports/cooperative-relay-native.json').write_text(json.dumps(report,indent=2)+'\n')


def engage(w,arms,staged_closure=False):
    pos,rot=w.pose('panel');targets={a:(pos+rot@GRIPS[a],rot@ROTATIONS[a]) for a in arms}
    w.phase('Approach panel edges '+','.join(str(a+1) for a in arms),3,
      {a:(p-r[:,2]*.05,r) for a,(p,r) in targets.items()})
    w.phase('Engage panel edges '+','.join(str(a+1) for a in arms),2.5,targets,touch={a:'panel' for a in arms})
    # Opposing approaches can overlap, but receiving contact is verified one
    # arm at a time. The already-established supports remain under load.
    for closing in ([[a] for a in arms] if staged_closure else [arms]):
        name='Arms 1 and 3 establish donor grips' if closing==[0,2] else f'Arm {closing[0]+1} establish receiver grip'
        w.phase(name,1.5,grippers={a:0 for a in closing},touch={a:'panel' for a in arms},
          gates=[grasp(a) for a in closing]+[UPRIGHT])
        w.holds.update({a:'panel' for a in closing})


def retreat(w,arms,name):
    w.phase(name,2.5,{a:(w.tcp(a)-w.rot(a)[:,2]*.14,w.rot(a)) for a in arms},touch={a:'panel' for a in arms})
    w.phase(name+' - home',3,joint_targets={a:HOME for a in arms})


def run(w):
    engage(w,[0,2]);w.qualify=True
    pos,_=w.pose('panel');w.coupled('Arms 1 and 3 lift panel together',pos+[0,0,.16],4)
    w.coupled('Move panel to shared relay pose',[0,0,.36],3)
    # Establish BOTH opposing receivers before either donor releases. One
    # receiver plus one adjacent donor creates an unbalanced support diagonal.
    engage(w,[1,3],staged_closure=True)
    for donor,receiver in [(0,1),(2,3)]:
        others={a:o for a,o in w.holds.items() if a!=donor}
        w.phase(f'Arm {donor+1} release to Arm {receiver+1}',1.5,grippers={donor:255},carry=others,gates=[grasp(a) for a in others]+[UPRIGHT])
        del w.holds[donor]
    # Open sequentially (4 -> 3 -> 2 verified supports), then withdraw the
    # opposing empty hands together. Contact speeds/forces stay unchanged.
    retreat(w,[0,2],'Arms 1 and 3 clear relay')
    w.coupled('Arms 2 and 4 carry panel over barrier',[0,.38,.30],5)
    w.coupled('Arms 2 and 4 lower onto output',[0,.38,.221],3)
    support={'type':'support','object':'panel','body':'panel_output'}
    w.phase('Settle panel at output',.8)
    for _ in range(5):
        if w.gate(support)[0]:break
        pos,_=w.pose('panel');w.coupled('Seek panel output support',pos-[0,0,.002],.6)
    w.phase('Confirm panel output support',.5,gates=[support,UPRIGHT])
    w.phase('Arms 2 and 4 release supported panel',1.5,grippers={1:255,3:255},carry={},gates=[support,UPRIGHT,{'type':'released','object':'panel'}])
    w.holds={};retreat(w,[1,3],'Arms 2 and 4 clear output')
    w.phase('Panel relay complete - all arms home',1,gates=[support,UPRIGHT,{'type':'released','object':'panel'},{'type':'home'}])
    w.export()


def failure_snapshot(w,error):
    contacts=base.Workcell.contacts(w)
    return {'error':str(error),'history':w.history,'phase':w.phases[-1]['name'] if w.phases else 'initial',
      'tcp':[w.tcp(a).tolist() for a in range(4)],'qpos':w.d.qpos.tolist(),'jointTargets':w.q.tolist(),
      'gripperTargets':w.grip.tolist(),'holds':w.holds,'panel':w.pose('panel')[0].tolist(),
      'contacts':[{'a':w.m.body(a).name,'b':w.m.body(b).name,'distance':float(c.dist)} for a,b,c in contacts]}


if __name__=='__main__':
    w=RelayWorkcell()
    try:run(w)
    except Exception as error:
        Path('artifacts/reports/relay-native-failure.json').write_text(json.dumps(failure_snapshot(w,error),indent=2)+'\n')
        raise
