#!/usr/bin/env python3
"""Checked multi-arm paths in one physical world; only actuators are driven."""
import argparse
import importlib.util
import json
from pathlib import Path
import mujoco
import numpy as np

spec=importlib.util.spec_from_file_location('single',Path(__file__).with_name('solve-egg-transfer.py'))
base=importlib.util.module_from_spec(spec);spec.loader.exec_module(base)
HOME=base.HOME
ORDERS=[[0,7,13,10],[4,1,11,14],[8,15,2,5],[12,3,9,6]]

def schedule(tasks):
    """Reserve source-to-clear corridors; peripheral tray work stays concurrent."""
    starts=[0.]*len(tasks);leases=[];available=[0.]*4
    for i,t in sorted(enumerate(tasks),key=lambda pair:(pair[1]['cellIndex'],[0,2,1,3].index(pair[1]['arm']))):
        length=sum(p['duration'] for p in t['phases'][:5]);start=available[t['arm']]
        while True:
            blocked=[end for begin,end,other in leases if start<end-1e-9 and start+length>begin+1e-9
                and not ((t['eggIndex']//4-t['arm'])%4<=1 and (other['eggIndex']//4-other['arm'])%4<=1
                    and abs(t['arm']-other['arm'])==2 and abs(t['eggIndex']//4-other['eggIndex']//4)==2)]
            if not blocked:break
            start=max(blocked)
        starts[i]=start;leases.append((start,start+length,t));available[t['arm']]=start+sum(p['duration'] for p in t['phases'])
    return starts

def rz(angle):
    c,s=np.cos(angle),np.sin(angle)
    return np.array([[c,-s,0],[s,c,0],[0,0,1.]])

def tray_load_bearing(o):
    return o['traySupported'] and o['fingerContacts']>=1 and o['cellError']<=.003 and o['tiltDegrees']<=20 and o['speed']<=.008

class Sorting:
    solve=base.Transfer.solve
    move=base.Transfer.move

    def __init__(self,root,checkpoint=None):
        self.model=m=mujoco.MjModel.from_xml_path(str(root/'public/assets/franka-egg-sorting/scene.xml'))
        self.data=d=mujoco.MjData(m);self.planner=mujoco.MjData(m)
        self.allj=[[m.joint(f'r{a}_joint{j+1}').id for j in range(7)] for a in range(4)]
        self.allqa=m.jnt_qposadr[self.allj];self.allda=m.jnt_dofadr[self.allj]
        self.Q=np.tile(HOME,(4,1));self.G=np.full(4,255.)
        for a in range(4):d.qpos[self.allqa[a]]=HOME;d.ctrl[a*8:a*8+8]=[*HOME,255]
        self.eggs=[m.body(f'egg_{i}').id for i in range(16)]
        self.trays=[m.body(f'output_tray_{i}').id for i in range(4)]
        self.sources={m.body(f'source_insert_{i}').id for i in range(4)}
        self.fingerSets=[{m.body(f'r{a}_{side}_finger').id for side in ['left','right']} for a in range(4)]
        self.robotBodies={i for i in range(m.nbody) if m.body(i).name.startswith(tuple(f'r{a}_' for a in range(4)))}
        self.active={};self.done={};self.tasks=[];self.phases=[];self.records=[]
        self.metrics={'maxForbiddenPenetration':0.,'maxGripPenetration':0.,'maxNeighborDisplacement':0.,'parallelSeconds':0.,'maxSimultaneousArms':0,'longestGripLoss':0.,'worstContact':None}
        self.loss={}
        self.initial=None;self.arm=0;self.select(0)
        if checkpoint:
            # Offline generation resumes a saved physical state as its initial
            # condition. Published playback always starts from the scene itself.
            d.qpos[:]=checkpoint['qpos'];d.qvel[:]=checkpoint['qvel'];d.time=checkpoint['time']
            self.Q[:]=checkpoint['Q'];self.G[:]=checkpoint['G'];self.initial=np.array(checkpoint['initial'])
            self.tasks=checkpoint['tasks'];self.done={t['eggIndex']:t for t in self.tasks};self.records=checkpoint['records']
            self.metrics=checkpoint['metrics'];mujoco.mj_forward(m,d)
        else:
            self.step(1.,audit=False)
            self.initial=d.xpos[self.eggs].copy()

    @property
    def q(self):return self.Q[self.arm]
    @q.setter
    def q(self,v):self.Q[self.arm]=v
    @property
    def grip(self):return self.G[self.arm]
    @grip.setter
    def grip(self,v):self.G[self.arm]=v

    def select(self,arm):
        self.arm=arm;self.joints=self.allj[arm];self.qa=self.allqa[arm];self.da=self.allda[arm]
        self.site=self.model.site(f'r{arm}_tcp').id

    def observe(self,task):
        m,d=self.model,self.data;index=task['eggIndex'];egg=self.eggs[index];a=task['arm']
        touching=set()
        for c in d.contact:
            if c.dist<=0:
                b1,b2=m.geom_bodyid[c.geom1],m.geom_bodyid[c.geom2]
                if egg in (b1,b2):touching.add(b2 if b1==egg else b1)
        p=d.xpos[egg];tcp=d.site_xpos[m.site(f'r{a}_tcp').id]
        return {'bilateral':self.fingerSets[a]<=touching,'fingerContacts':len(self.fingerSets[a]&touching),
            'traySupported':self.trays[a] in touching,'lift':float(p[2]-self.initial[index,2]),
            'tiltDegrees':float(np.degrees(np.arccos(np.clip(d.xmat[egg,8],-1,1)))),
            'cellError':float(np.linalg.norm(p[:2]-np.array(task['cell'])[:2])),
            'speed':float(np.linalg.norm(d.joint(f'egg_{index}_free').qvel[:3])),
            'tcpDistance':float(np.linalg.norm(p-tcp)),
            'aperture':sum(float(d.joint(f'r{a}_finger_joint{j}').qpos[0]) for j in [1,2])}

    def audit(self):
        m,d=self.model,self.data
        allowed={}
        for index,egg in enumerate(self.eggs):
            if index in self.done:allowed[egg]={self.trays[self.done[index]['arm']]}
            else:allowed[egg]=self.sources.copy()
        for a,t in self.active.items():
            supports=self.sources if t['stage']<=3 else {self.trays[a]} if t['stage']>=6 else set()
            allowed[self.eggs[t['eggIndex']]]=supports|self.fingerSets[a]
        for c in d.contact:
            if c.dist>=0:continue
            a,b=m.geom_bodyid[c.geom1],m.geom_bodyid[c.geom2]
            forbidden=(a in self.robotBodies or b in self.robotBodies)
            if a in allowed:forbidden=b not in allowed[a]
            if b in allowed:forbidden=forbidden or a not in allowed[b] if a in allowed else a not in allowed[b]
            if forbidden and -c.dist>self.metrics['maxForbiddenPenetration']:
                self.metrics['maxForbiddenPenetration']=float(-c.dist)
                self.metrics['worstContact']=[m.body(a).name,m.body(b).name]
            if not forbidden and (a in self.robotBodies or b in self.robotBodies):
                self.metrics['maxGripPenetration']=max(self.metrics['maxGripPenetration'],float(-c.dist))
        if self.initial is not None:
            untouched=[i for i in range(16) if i not in self.done and i not in [t['eggIndex'] for t in self.active.values()]]
            if untouched:self.metrics['maxNeighborDisplacement']=max(self.metrics['maxNeighborDisplacement'],float(np.max(np.linalg.norm(d.xpos[np.array(self.eggs)[untouched]]-self.initial[untouched],axis=1))))
        if self.metrics['maxForbiddenPenetration']>.0001 or self.metrics['maxGripPenetration']>.001:
            raise RuntimeError(f'Contact failure at {d.time:.3f}: {self.metrics}, active {[(a,t["eggIndex"],t["stage"]) for a,t in self.active.items()]}')
        for a,t in self.active.items():
            gate=t['phases'][t['stage']]['gate'] if t['phases'] else None
            o=self.observe(t)
            if (gate=='carried' or (gate=='supported' and not tray_load_bearing(o))) and not o['bilateral']:
                self.loss.setdefault(a,d.time)
                elapsed=float(d.time-self.loss[a]);self.metrics['longestGripLoss']=max(self.metrics['longestGripLoss'],elapsed)
                if elapsed>.12:raise RuntimeError(f'Lost physical grip arm {a}, egg {t["eggIndex"]}')
            else:self.loss.pop(a,None)

    def step(self,seconds,callback=None,audit=True):
        m,d=self.model,self.data;ticks=round(seconds/m.opt.timestep)
        for tick in range(ticks):
            old=self.Q.copy()
            if callback:callback((tick+1)/ticks)
            moving=int(np.sum(np.linalg.norm(self.Q-old,axis=1)>1e-8))
            self.metrics['maxSimultaneousArms']=max(self.metrics['maxSimultaneousArms'],moving)
            if moving>=2:self.metrics['parallelSeconds']+=m.opt.timestep
            for a in range(4):
                d.ctrl[a*8:a*8+7]=self.Q[a]+d.qfrc_bias[self.allda[a]]/m.actuator_gainprm[a*8:a*8+7,0]
                d.ctrl[a*8+7]=self.G[a]
            mujoco.mj_step(m,d)
            if audit and tick%5==0:self.audit()
        if audit:self.audit()

    def gate(self,t,gate):
        o=self.observe(t);reason=None
        if gate in ('bilateral','carried'):
            if not o['bilateral']:reason='missing-two-sided-contact'
            elif not .028<o['aperture']<.070:reason='grip-aperture'
            elif o['tcpDistance']>.065:reason='not-carried'
        if gate=='carried' and o['lift']<.06:reason='insufficient-lift'
        if gate=='supported' and (not tray_load_bearing(o) or not .028<o['aperture']<.070 or o['tcpDistance']>.065):reason='invalid-tray-load-transfer'
        if gate in ('supported','released','seated') and not o['traySupported']:reason='unsupported'
        if gate in ('released','seated') and o['fingerContacts']:reason='not-released'
        if gate=='seated' and (o['tiltDegrees']>20 or o['cellError']>.012 or o['speed']>.008):reason='not-seated'
        if reason:
            a=t['arm'];m,d=self.model,self.data
            print(json.dumps({'failureDiagnostic':reason,'gripCommand':float(d.ctrl[a*8+7]),'gripForce':float(d.actuator_force[a*8+7]),
                'fingerContacts':[(m.geom(c.geom1).name,m.geom(c.geom2).name,float(c.dist)) for c in d.contact if c.dist<.002 and (m.geom_bodyid[c.geom1] in self.fingerSets[a] or m.geom_bodyid[c.geom2] in self.fingerSets[a])]}),flush=True)
            raise RuntimeError(f'Gate {reason} arm {t["arm"]} egg {t["eggIndex"]}: {o}')
        return o

    def phase(self,name,target,duration,grip=None,gate=None,path=None):
        t=self.active[self.arm];start=self.q.copy();start_grip=self.grip
        end_grip=self.grip if grip is None else grip
        path=np.array([start,target] if path is None else path)
        phase={'name':name,'duration':duration,'path':path.tolist(),'gripper':float(end_grip),'gate':gate}
        t['stage']=len(t['phases']);t['phases'].append(phase)
        def update(f):
            s=f*f*f*(10+f*(-15+6*f));k=min(int(s*(len(path)-1)),len(path)-2);r=s*(len(path)-1)-k
            self.q=path[k]+(path[k+1]-path[k])*r;self.grip=start_grip+(end_grip-start_grip)*s
        self.step(duration,update)
        o=self.gate(t,gate)
        self.records.append({'arm':self.arm,'egg':t['eggIndex'],'phase':name,**o})
        print(json.dumps(self.records[-1]),flush=True)

    def support_height(self,index,cell):
        # Independent planning experiment, initialized once above an empty cell.
        # Never changes this controller's live data or any runtime object pose.
        m=self.model;probe=mujoco.MjData(m)
        for a in range(4):
            probe.qpos[self.allqa[a]]=HOME;probe.ctrl[a*8:a*8+8]=[*HOME,255]
        probe.joint(f'egg_{index}_free').qpos[:]=[*cell[:2],.17,1,0,0,0]
        for _ in range(2000):mujoco.mj_step(m,probe)
        egg=self.eggs[index]
        return float(probe.xpos[egg,2])-.0001

    def seating_rotation(self,cell,rotation,egg):
        """Choose an upright-equivalent yaw that clears already occupied cells."""
        if not self.done:return rotation
        m,d,p=self.model,self.data,self.planner
        other_geoms=[i for i in range(m.ngeom) if m.geom_bodyid[i] in [self.eggs[e] for e in self.done]]
        finger_geoms=[i for i in range(m.ngeom) if m.geom_bodyid[i] in self.fingerSets[self.arm] and (m.geom_contype[i] or m.geom_conaffinity[i])]
        current=d.site_xmat[self.site].reshape(3,3);offset=d.xpos[egg]-d.site_xpos[self.site]
        choices=[]
        for yaw in [0,*[sign*np.radians(deg) for deg in range(15,181,15) for sign in [1,-1]]]:
            R=rz(yaw)@rotation;target=np.array(cell)-(R@current.T)@offset
            try:q=self.solve(target,R,self.q)
            except RuntimeError:continue
            p.qpos[:]=d.qpos;p.qpos[self.qa]=q;mujoco.mj_forward(m,p)
            distance=min((mujoco.mj_geomDistance(m,p,a,b,.04,None) for a in finger_geoms for b in other_geoms),default=.04)
            choices.append((float(distance),yaw,R))
        valid=[c for c in choices if c[0]>.004]
        if not valid:raise RuntimeError(f'No clear seating yaw arm {self.arm}: {[(c[0],c[1]) for c in choices]}')
        chosen=min(valid,key=lambda c:abs(c[1]))
        print(json.dumps({'seatingYawArm':self.arm,'yawDegrees':float(np.degrees(chosen[1])),'clearance':chosen[0]}),flush=True)
        return chosen[2]

    def grasp_rotation(self,pos,rotation,index,opening,cell):
        m,d,p=self.model,self.data,self.planner
        fingers=[g for g in range(m.ngeom) if m.geom_bodyid[g] in self.fingerSets[self.arm] and (m.geom_contype[g] or m.geom_conaffinity[g])]
        obstacles=[g for g in range(m.ngeom) if m.geom_bodyid[g] in self.sources and (m.geom_contype[g] or m.geom_conaffinity[g])]
        egg_geom=m.geom(f'egg_{index}_shell').id
        candidates=[]
        options=[(height,yaw) for height in [.025,.030,.035] for yaw in [0,*[sign*np.radians(deg) for deg in range(15,181,15) for sign in [1,-1]]]]
        for height,yaw in options:
            R=rotation@rz(yaw);target=pos-R[:,2]*height
            try:q=self.solve(target,R,self.q)
            except RuntimeError:continue
            p.qpos[:]=d.qpos;p.qpos[self.qa]=q
            for j in [1,2]:p.joint(f'r{self.arm}_finger_joint{j}').qpos[0]=opening/255*.05
            mujoco.mj_forward(m,p)
            distance=.04
            for a in fingers:
                near=[b for b in obstacles if np.linalg.norm(p.geom_xpos[a]-p.geom_xpos[b])<m.geom_rbound[a]+m.geom_rbound[b]+.01]
                distance=min(distance,*[mujoco.mj_geomDistance(m,p,a,b,.04,None) for b in near]) if near else distance
            gap=min(mujoco.mj_geomDistance(m,p,a,egg_geom,.04,None) for a in fingers)
            candidates.append((distance,gap,yaw,height))
            if distance>.001 and gap>.001:
                # A grasp is not useful if its wrist branch cannot exit and
                # reach the assigned tray. Check that continuation before pick.
                exit_pos=target+[0,0,.13];tray_pos=np.array(cell)-R[:,2]*height+[0,0,.16]
                try:
                    seed=self.solve(exit_pos,R,q)
                    for f in np.linspace(0,1,9)[1:]:seed=self.solve(exit_pos+(tray_pos-exit_pos)*f,R,seed)
                except RuntimeError:continue
                print(json.dumps({'graspYawArm':self.arm,'egg':index,'yaw':float(np.degrees(yaw)),'clearance':float(distance),'eggGap':float(gap),'graspDepth':height}),flush=True)
                return R,target
        raise RuntimeError(f'No clear source grasp {self.arm}/{index}: {candidates}')

    def refine_landing(self,t):
        """Offline IK against the concurrent run's actual grasp offset.

        Only planned arm paths change. A separate fresh fixed-path replay must
        pass before export; the browser does not run this optimizer.
        """
        self.select(t['arm']);m,d,p=self.model,self.data,self.planner
        R=d.site_xmat[self.site].reshape(3,3).copy()
        target=np.array(t['cell'])-(d.xpos[self.eggs[t['eggIndex']]]-d.site_xpos[self.site])
        seed=np.array(t['phases'][5]['path'][-1])
        def path_to(seed,target):
            p.qpos[:]=d.qpos;p.qpos[self.qa]=seed;mujoco.mj_kinematics(m,p)
            start=p.site_xpos[self.site].copy();path=[seed.copy()]
            for f in np.linspace(0,1,max(12,int(np.linalg.norm(target-start)/.012))+1)[1:]:
                path.append(self.solve(start+(target-start)*f,R,path[-1]))
            return np.array(path)
        release_rise=.012 if t['phases'][7]['gripper']==160 else 0.
        for stage,destination in [(6,target),(7,target+[0,0,release_rise]),(8,target+[0,0,.16])]:
            path=path_to(seed,np.array(destination));t['phases'][stage]['path']=path.tolist();seed=path[-1]
        t['phases'][9]['path']=[seed.tolist(),HOME.tolist()]
        print(json.dumps({'refinedLanding':t['eggIndex'],'arm':t['arm'],'target':target.tolist()}),flush=True)

    def make_task(self,arm,egg_index,cell_index):
        self.select(arm);m,d=self.model,self.data
        cell=(rz(arm*np.pi/2)@np.array([[-.442,-.442,.1463],[-.358,-.442,.1463],[-.442,-.358,.1463],[-.358,-.358,.1463]][cell_index])).tolist()
        cell[2]=self.support_height(egg_index,cell)
        t={'arm':arm,'eggIndex':egg_index,'classIndex':arm,'cellIndex':cell_index,'cell':cell,'phases':[],'stage':0}
        self.active[arm]=t
        c=2**-.5;down=rz(arm*np.pi/2)@np.array([[-c,-c,0],[-c,c,0],[0,0,-1.]])
        egg=self.eggs[egg_index];pos=d.xpos[egg].copy()
        # Far rows exceed vertical-wrist reach. Tilt the sourced long fingers
        # toward the egg instead of stretching links or changing the base.
        forward=(rz(-arm*np.pi/2)@pos)[1]
        angle=np.radians(25 if forward>.09 else 15 if forward>.015 else 0)
        rx=np.array([[1,0,0],[0,np.cos(angle),-np.sin(angle)],[0,np.sin(angle),np.cos(angle)]])
        down=rz(arm*np.pi/2)@rx@rz(-arm*np.pi/2)@down
        grasp=pos-down[:,2]*.025
        opening=140 if angle else 160
        if angle:down,grasp=self.grasp_rotation(pos,down,egg_index,opening,cell)
        self.move('Approach egg',grasp+[0,0,.13 if angle else .176],down,3.,grip=opening)
        self.move('Lower fingers',grasp,down,3.)
        self.phase('Close on egg',self.q,2.,grip=0,gate='bilateral')
        self.move('Lift clear',grasp+[0,0,.13],down,3.,gate='carried')
        offset=d.xpos[egg]-d.site_xpos[self.site]
        self.move('Transfer over tray',np.array(cell)-offset+[0,0,.16],down,4.,gate='carried')
        axis=d.xmat[egg].reshape(3,3)[:,2];v=np.cross(axis,[0,0,1.])
        skew=np.array([[0,-v[2],v[1]],[v[2],0,-v[0]],[-v[1],v[0],0]])
        correction=(np.eye(3)+skew+skew@skew/(1+axis[2]))@d.site_xmat[self.site].reshape(3,3)
        correction=self.seating_rotation(cell,correction,egg)
        self.move('Align upright',d.site_xpos[self.site].copy(),correction,3.,gate='carried')
        offset=d.xpos[egg]-d.site_xpos[self.site]
        self.move('Lower into tray',np.array(cell)-offset,correction,4.,gate='supported')
        if angle and cell_index==3:
            # A tilted wrist's outer fingertip can meet the tray rim while
            # opening in the last, surrounded corner. Earlier cells use the
            # checked narrower in-place release; here exit upward while opening.
            self.move('Release egg',d.site_xpos[self.site].copy()+[0,0,.012],correction,2.,grip=160,gate='released')
        else:self.phase('Release egg',self.q,2.,grip=130,gate='released')
        self.move('Withdraw',np.array(cell)-offset+[0,0,.16],correction,3.,grip=255)
        self.phase('Return home',HOME,4.)
        self.phase('Verify placement',self.q,2.,gate='seated')
        self.done[egg_index]=t;del self.active[arm];self.tasks.append(t)
        return t

def build_candidates(root,arms=2,rounds=1,resume=False):
    checkpoint=root/'artifacts/reports/demo2-sorting-checkpoint.json'
    sim=Sorting(root,json.loads(checkpoint.read_text()) if resume and checkpoint.exists() else None)
    for r in range(rounds):
        for a in range(arms):
            if ORDERS[a][r] in sim.done:continue
            sim.make_task(a,ORDERS[a][r],r)
            checkpoint.write_text(json.dumps({'qpos':sim.data.qpos.tolist(),'qvel':sim.data.qvel.tolist(),'time':sim.data.time,
                'Q':sim.Q.tolist(),'G':sim.G.tolist(),'initial':sim.initial.tolist(),'tasks':sim.tasks,'records':sim.records,'metrics':sim.metrics}))
    return {'schemaVersion':1,'program':'sorting','success':len(sim.done)==arms*rounds,
        'counts':[sum(t['arm']==a for t in sim.done.values()) for a in range(4)],
        'initialEggPositions':sim.initial.tolist(),'tasks':sim.tasks,'metrics':sim.metrics,'records':sim.records}

def build_and_verify(root,arms=2,rounds=1):
    candidate=build_candidates(root,arms,rounds)
    starts=schedule(candidate['tasks'])
    return replay_timeline(root,candidate['tasks'],starts)

def replay(root,tasks,batches):
    """Whole-world simultaneous physical validation, not independent-arm scores."""
    sim=Sorting(root)
    for batch_index,batch in enumerate(batches):
        current=[dict(tasks[i]) for i in batch]
        for t in current:sim.active[t['arm']]=t
        for stage in range(11):
            starts=sim.G.copy();duration=max(t['phases'][stage]['duration'] for t in current)
            for t in current:t['stage']=stage
            def update(f):
                for t in current:
                    p=t['phases'][stage];a=t['arm'];u=min(1,f*duration/p['duration']);s=u*u*u*(10+u*(-15+6*u))
                    path=np.array(p['path']);k=min(int(s*(len(path)-1)),len(path)-2);r=s*(len(path)-1)-k
                    sim.Q[a]=path[k]+(path[k+1]-path[k])*r
                    sim.G[a]=starts[a]+(p['gripper']-starts[a])*s
            sim.step(duration,update)
            for t in current:
                o=sim.gate(t,t['phases'][stage]['gate'])
                record={'batch':batch_index,'stage':stage,'arm':t['arm'],'egg':t['eggIndex'],**o}
                sim.records.append(record);print(json.dumps(record),flush=True)
        for t in current:sim.done[t['eggIndex']]=t;del sim.active[t['arm']]
    for t in sim.done.values():sim.gate(t,'seated')
    return {'schemaVersion':1,'program':'sorting','success':True,'initialJoints':HOME.tolist(),
        'initialEggPositions':sim.initial.tolist(),'tasks':tasks,'batches':batches,
        'counts':[sum(t['arm']==a for t in sim.done.values()) for a in range(4)],'metrics':sim.metrics,'records':sim.records}

def replay_timeline(root,tasks,starts,refine=False):
    sim=Sorting(root);events=[];scheduled=[]
    for task,start in zip(tasks,starts):
        t=dict(task,start=float(start),stage=0);scheduled.append(t)
        events.append((float(start),'start',t,0))
        end=float(start)
        for stage,p in enumerate(t['phases']):
            end+=p['duration'];events.append((end,'end',t,stage))
    times=sorted(set(e[0] for e in events));now=0.
    for boundary in times:
        def update(f):
            time=now+(boundary-now)*f
            for a,t in sim.active.items():
                local=time-t['start'];offset=0.;start_grip=255.
                for stage,p in enumerate(t['phases']):
                    if local<=offset+p['duration']+1e-9:break
                    offset+=p['duration'];start_grip=p['gripper']
                u=np.clip((local-offset)/p['duration'],0,1);s=u*u*u*(10+u*(-15+6*u))
                path=np.array(p['path']);k=min(int(s*(len(path)-1)),len(path)-2);r=s*(len(path)-1)-k
                sim.Q[a]=path[k]+(path[k+1]-path[k])*r;sim.G[a]=start_grip+(p['gripper']-start_grip)*s
        if boundary>now:sim.step(boundary-now,update)
        # Complete before starting another task on the same arm at this boundary.
        for _,kind,t,stage in sorted([e for e in events if e[0]==boundary],key=lambda e:e[1]):
            if kind=='end':
                o=sim.gate(t,t['phases'][stage]['gate'])
                record={'time':boundary,'stage':stage,'arm':t['arm'],'egg':t['eggIndex'],**o}
                sim.records.append(record);print(json.dumps(record),flush=True)
                if refine and stage==5:sim.refine_landing(t)
                if stage==10:sim.done[t['eggIndex']]=t;sim.active.pop(t['arm'],None)
                else:t['stage']=stage+1
            else:
                if t['arm'] in sim.active:raise RuntimeError('Overlapping tasks own the same arm')
                sim.active[t['arm']]=t
        now=boundary
    for t in sim.done.values():sim.gate(t,'seated')
    home_error=float(np.max(np.abs(sim.data.qpos[sim.allqa]-HOME)))
    warnings=[int(w.number) for w in sim.data.warning]
    if home_error>.025 or any(warnings):raise RuntimeError(f'Final robot state: {home_error}, warnings {warnings}')
    if sim.metrics['maxNeighborDisplacement']>.002:raise RuntimeError(f'Fresh replay disturbed an unpicked egg: {sim.metrics}')
    return {'schemaVersion':1,'program':'sorting','success':True,'initialJoints':HOME.tolist(),
        'initialEggPositions':sim.initial.tolist(),'tasks':scheduled,'duration':now,
        'counts':[sum(t['arm']==a for t in sim.done.values()) for a in range(4)],'metrics':sim.metrics,'records':sim.records,
        'warnings':warnings,'maxHomeError':home_error,'engine':mujoco.mj_versionString(),'refinedDuringReplay':refine}

def export_motion(root,result):
    assert not result.get('refinedDuringReplay'), 'Export requires a fresh fixed-path replay after refinement'
    assert result['success'] and result['counts']==[4,4,4,4] and len(result['tasks'])==16
    assert result['metrics']['maxForbiddenPenetration']<=.0001 and result['metrics']['maxGripPenetration']<=.001
    assert result['metrics']['maxNeighborDisplacement']<=.002 and result['metrics']['maxSimultaneousArms']==4
    assert result['maxHomeError']<.025 and not any(result['warnings'])
    motion={k:result[k] for k in ['schemaVersion','program','success','initialJoints','initialEggPositions','tasks','duration','counts']}
    (root/'public/assets/franka-egg-sorting/sorting-motion.json').write_text(json.dumps(motion,indent=2)+'\n')

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--arms',type=int,default=2);p.add_argument('--rounds',type=int,default=1)
    p.add_argument('--replay',action='store_true');p.add_argument('--stagger',type=float);p.add_argument('--resume',action='store_true');p.add_argument('--export',action='store_true')
    p.add_argument('--refine',action='store_true')
    p.add_argument('--input',default='artifacts/reports/demo2-sorting-candidates.json');p.add_argument('--limit',type=int,default=16);args=p.parse_args()
    root=Path(__file__).resolve().parents[1]
    if args.replay:
        tasks=json.loads((root/args.input).read_text())['tasks'][:args.limit]
        if args.stagger is not None:
            starts=[(i//args.arms)*sum(p['duration'] for p in t['phases'])+(t['arm']%2)*args.stagger for i,t in enumerate(tasks)]
            result=replay_timeline(root,tasks,starts)
        else:result=replay_timeline(root,tasks,schedule(tasks),args.refine)
        report='demo2-sorting-parallel.json' if args.arms==2 else 'demo2-four-arm-native.json'
        (root/'artifacts/reports'/report).write_text(json.dumps(result,indent=2)+'\n')
        if args.export:export_motion(root,result)
    else:
        result=build_candidates(root,args.arms,args.rounds,args.resume)
        (root/'artifacts/reports/demo2-sorting-candidates.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({'counts':result['counts'],'metrics':result['metrics']}),flush=True)
