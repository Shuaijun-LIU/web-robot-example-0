#!/usr/bin/env python3
"""Plan and verify Demo5 on CPU. Only IK scratch data is pose-written."""
import importlib.util
import json
from pathlib import Path
import numpy as np
import mujoco

spec=importlib.util.spec_from_file_location('cooperative_solver',Path(__file__).with_name('solve-cooperative-tasks.py'))
base=importlib.util.module_from_spec(spec);spec.loader.exec_module(base)
HOME,ROOT,top_rotation=base.HOME,base.ROOT,base.top_rotation


class DrawerWorkcell(base.Workcell):
    def __init__(self):
        super().__init__('drawer')
        self.initial_fixture=[{'joint':n,'position':float(self.d.joint(n).qpos[0]),'tolerance':.006} for n in ['south_slide','north_slide']]

    def gate(self,g):
        if g['type']=='joint-range':
            j=self.d.joint(g['joint']);o={'position':float(j.qpos[0]),'speed':abs(float(j.qvel[0]))}
            return g['min']<=o['position']<=g['max'] and o['speed']<=g['maxSpeed'],o
        passed,obs=super().gate(g)
        if g['type']=='inside' and 'minX' in g:passed=passed and g['minX']<=obs['relative'][0]<=g['maxX']
        return passed,obs

    def export(self):
        plan={'version':1,'scene':'drawer','initialJoints':HOME.tolist(),'initialObjects':self.initial,
          'initialFixtureJoints':self.initial_fixture,'phases':self.phases}
        (ROOT/'drawer-motion.json').write_text(json.dumps(plan,separators=(',',':'))+'\n')
        report={'success':True,'engine':mujoco.mj_versionString(),'maxForbiddenPenetration':self.max_forbidden,
          'maxGripPenetration':self.max_grip,'history':self.history,
          'fixtureJoints':{n:float(self.d.joint(n).qpos[0]) for n in ['south_slide','north_slide']}}
        Path('artifacts/reports/cooperative-drawer-native.json').write_text(json.dumps(report,indent=2)+'\n')


OPERATORS=[(0,'south',-1),(2,'north',1)]


def joint_gate(prefix,opened):
    return {'type':'joint-range','joint':prefix+'_slide','min':-.22 if opened else -.006,
      'max':-.175 if opened else .006,'maxSpeed':.015}


def handle_cycle(w,opened):
    verb='Open' if opened else 'Close'
    points={a:w.d.site(f'{prefix}_grasp').xpos.copy() for a,prefix,sign in OPERATORS}
    # Pitch toward the front: vertical approach hits the roof, while a fully
    # horizontal wrist forces joint 6 to its limit during the home transition.
    c,s=np.cos(.85),np.sin(.85)
    # Symmetric jaws permit a half-turn around the approach axis; this branch
    # keeps wrist joint 7 near its center while preserving the same grasp.
    rotations={a:np.array([[-sign,0,0],[0,sign*c,-sign*s],[0,-s,-c]]) for a,_,sign in OPERATORS}
    touching={a:prefix+'_handle' for a,prefix,_ in OPERATORS}
    if opened:
        w.phase(f'{verb}: approach both handles',3,{a:(p-rotations[a][:,2]*.14,rotations[a]) for a,p in points.items()},
          grippers={a:110 for a in points},touch=touching)
    else:
        # Re-enter on the already verified opening branch. Solving directly
        # from HOME can choose an elbow branch that cannot finish the push.
        w.phase(f'{verb}: approach both handles',3,joint_targets=w.open_retracted,
          grippers={a:110 for a in points},touch=touching)
    w.phase(f'{verb}: enter handle clearance',2.5,{a:(p,rotations[a]) for a,p in points.items()},touch=touching)
    w.phase(f'{verb}: grasp both handles',1.5,grippers={a:0 for a in points},touch=touching,
      gates=[{'type':'grasp','object':obj,'arm':a} for a,obj in touching.items()])
    w.holds=touching.copy()
    targets={}
    for a,prefix,sign in OPERATORS:
        current=float(w.d.joint(prefix+'_slide').qpos[0]);desired=-.205 if opened else -.001
        targets[a]=(w.tcp(a)+[0,-sign*(desired-current),0],rotations[a])
    w.phase(f'{verb}: slide both drawers through handle contact',5,targets,
      gates=[joint_gate(prefix,opened) for _,prefix,_ in OPERATORS])
    w.phase(f'{verb}: confirm drawers at rest',.6,gates=[joint_gate(prefix,opened) for _,prefix,_ in OPERATORS])
    w.phase(f'{verb}: release both handles',1.4,grippers={a:110 for a in points},carry={},
      gates=[{'type':'released','object':obj} for obj in touching.values()])
    w.holds={}
    w.phase(f'{verb}: withdraw above handles',2.5,{a:(w.tcp(a)-rotations[a][:,2]*.15,rotations[a]) for a in points},touch=touching)
    if opened:w.open_retracted={a:w.q[a].copy() for a in points}
    w.phase(f'{verb}: drawer operators return home',3,joint_targets={a:HOME for a in points},grippers={a:255 for a in points})


def slot(box):
    return {'type':'inside','object':box,'container':'order_tray','radius':.14,'minZ':.005,'maxZ':.09,
      'minX':.02 if box=='tea_box' else -.12,'maxX':.12 if box=='tea_box' else -.02}


def run(w):
    handle_cycle(w,True)
    for arm,box,z,yaw,dest in [(1,'tea_box',.045,-np.pi/2,.075),(3,'coffee_box',.075,np.pi/2,-.075)]:
        bp,_=w.pose(box);rot=top_rotation(yaw)
        w.pick(arm,box,bp+[0,0,z],rot)
        w.phase(f'Arm {arm+1} lift clear of cabinet roof',2,{arm:([*w.tcp(arm)[:2],.43],rot)})
        w.phase(f'Arm {arm+1} enter shared packing zone',3,{arm:([dest,0,.43],rot)})
        tray,_=w.pose('order_tray')
        w.place(arm,box,[dest,0],tray[2]+.009,'order_tray')
        w.phase(f'Confirm {box} in assigned order slot',.6,gates=[slot(box),{'type':'released','object':box}])
        w.phase(f'Arm {arm+1} leave shared packing zone',2,{arm:([dest,0,.43],rot)})
        w.phase(f'Arm {arm+1} return home',3,joint_targets={arm:HOME})
    handle_cycle(w,False)
    w.phase('Order complete - drawers closed and all arms home',1,gates=[slot('tea_box'),slot('coffee_box'),
      joint_gate('south',False),joint_gate('north',False),{'type':'home'},
      *[{'type':'released','object':o} for o in ['tea_box','coffee_box','south_handle','north_handle']]])
    w.export()


if __name__=='__main__':
    w=DrawerWorkcell()
    try:run(w)
    except Exception:
        Path('artifacts/reports/drawer-native-failure.json').write_text(json.dumps({
          'history':w.history,'phase':w.phases[-1]['name'] if w.phases else 'initial',
          'tcp':[w.tcp(a).tolist() for a in range(4)],
          'jointTargets':w.q.tolist(),'actualJoints':[w.d.qpos[qa].tolist() for qa in w.qa],
          'ctrl':w.d.ctrl.tolist(),'actuatorForces':w.d.actuator_force.tolist(),
          'maxForbiddenPenetration':w.max_forbidden,'maxGripPenetration':w.max_grip,
          'fixtures':{n:float(w.d.joint(n).qpos[0]) for n in ['south_slide','north_slide']},
          'contacts':[{'a':w.m.body(a).name,'b':w.m.body(b).name,'distance':float(c.dist)} for a,b,c in w.contacts()]},indent=2)+'\n')
        raise
