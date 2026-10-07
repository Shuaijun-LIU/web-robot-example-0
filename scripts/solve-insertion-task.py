#!/usr/bin/env python3
"""CPU-only real-contact demonstration using source robosuite stand/frame."""
import importlib.util
import json
from pathlib import Path
import numpy as np
import mujoco

spec=importlib.util.spec_from_file_location('cooperative_solver',Path(__file__).with_name('solve-cooperative-tasks.py'))
base=importlib.util.module_from_spec(spec);spec.loader.exec_module(base)
ROOT,HOME,top_rotation=base.ROOT,base.HOME,base.top_rotation
GEOMETRY=json.loads((ROOT/'insertion-manifest.json').read_text())


def inserted():
    return {'type':'inserted','object':'insert','socket':'stand','tip':GEOMETRY['tip'],'mouth':GEOMETRY['mouth'],
      'axis':[0,0,1],'socketAxis':[0,0,1],'minDepth':.095,'maxDepth':.13,'maxLateral':.006,'minAlignment':.98,'maxSpeed':.025}


class InsertionWorkcell(base.Workcell):
    def __init__(self):
        super().__init__('insertion');self.max_limited=0.
        for o in self.initial:o.update(quaternion=self.d.body(o['name']).xquat.tolist(),orientationTolerance=.12)

    def contacts(self):
        contacts=super().contacts();parts={self.m.body('stand').id,self.m.body('insert').id}
        for a,b,c in contacts:
            if {a,b}==parts:
                self.max_limited=max(self.max_limited,-float(c.dist))
                if c.dist<-.001:raise RuntimeError(f'Insertion pair penetration {-c.dist:.6f} m')
        return contacts

    def gate(self,g):
        if g['type']!='inserted':return super().gate(g)
        pos,rot=self.pose(g['object']);sp,sr=self.pose(g['socket'])
        axis=sr@np.array(g['socketAxis']);delta=pos+rot@np.array(g['tip'])-(sp+sr@np.array(g['mouth']))
        depth=-float(delta@axis);alignment=float((rot@np.array(g['axis']))@axis)
        id=self.m.body(g['object']).id;sid=self.m.body(g['socket']).id
        dof=self.m.jnt_dofadr[self.m.body_jntadr[id]]
        o={'depth':depth,'lateral':float(np.linalg.norm(delta+depth*axis)),'alignment':alignment,
          'speed':float(np.linalg.norm(self.d.qvel[dof:dof+3])),
          'supported':any({a,b}=={id,sid} for a,b,c in self.contacts())}
        return o['supported'] and g['minDepth']<=depth<=g['maxDepth'] and o['lateral']<=g['maxLateral'] and alignment>=g['minAlignment'] and o['speed']<=g['maxSpeed'],o

    def export(self):
        plan={'version':1,'scene':'insertion','initialJoints':HOME.tolist(),'initialObjects':self.initial,
          'contactLimits':[{'a':'stand','b':'insert','maxPenetration':.001}],'phases':self.phases}
        (ROOT/'insertion-motion.json').write_text(json.dumps(plan,separators=(',',':'))+'\n')
        report={'success':True,'engine':mujoco.mj_versionString(),'maxForbiddenPenetration':self.max_forbidden,
          'maxGripPenetration':self.max_grip,'maxLimitedContactPenetration':self.max_limited,'history':self.history}
        Path('artifacts/reports/cooperative-insertion-native.json').write_text(json.dumps(report,indent=2)+'\n')


def frame_grip(w):
    p,r=w.pose('insert');return p+r@np.array(GEOMETRY['grip'])+[0,0,.004]


def stand_grip(w):
    p,r=w.pose('stand');return p+r@np.array([0,-.052,-.0685])


def run(w):
    w.pick(2,'insert',frame_grip(w),top_rotation(-np.pi/2))
    w.place(2,'insert',[.25,.10],.1427,'insert_staging')
    w.phase('Arm 3 clear of insertion supply',3,joint_targets={2:HOME})
    point=stand_grip(w);rotation=top_rotation(np.pi/2)
    w.phase('Arm 1 approach stand base',3,{0:(point+[0,0,.12],rotation)},touch={0:'stand'})
    w.phase('Arm 1 engage stand base',2,{0:(point,rotation)},touch={0:'stand'})
    w.phase('Arm 1 stabilize stand',1.3,grippers={0:0},touch={0:'stand'},gates=[{'type':'grasp','object':'stand','arm':0}])
    w.holds[0]='stand'
    w.pick(1,'insert',frame_grip(w),top_rotation(np.pi/2))
    # The source L points west. This upright branch clears joint 5; trying
    # to turn the L north wrapped the wrist into its physical limit.
    upright=np.eye(3)
    w.move_object('Arm 2 orient frame upright',1,'insert',[.23,.10,.40],upright,4)
    sp,sr=w.pose('stand');mouth=sp+sr@np.array(GEOMETRY['mouth']);tip=np.array(GEOMETRY['tip'])
    rotation=sr@upright
    w.move_object('Arm 2 align above actual socket',1,'insert',mouth+[0,0,.035]-rotation@tip,rotation,3)
    # Lower until the source grip shoulder rests on the real four-wall rim.
    w.move_object('Arm 2 insert frame into socket',1,'insert',mouth+[0,0,-.107]-rotation@tip,rotation,4)
    w.phase('Confirm supported insertion',.8,gates=[inserted()])
    w.phase('Arm 2 release seated frame',1.4,grippers={1:255},carry={0:'stand'},gates=[inserted(),{'type':'released','object':'insert'}])
    del w.holds[1]
    w.phase('Arm 2 withdraw from frame',2,{1:(w.tcp(1)-w.rot(1)[:,2]*.04,w.rot(1))},touch={1:'insert'})
    w.phase('Arm 2 return toward its own side',2,{1:(w.tcp(1)+[.16,0,0],w.rot(1))})
    w.phase('Arm 2 clear assembly',3,joint_targets={1:HOME})
    w.phase('Arm 1 release stabilized stand',1.2,grippers={0:255},carry={},gates=[inserted(),{'type':'released','object':'stand'}])
    w.holds={}
    w.phase('Arm 1 withdraw from base',2,{0:(w.tcp(0)+[0,0,.12],top_rotation(np.pi/2))},touch={0:'stand'})
    w.phase('Arm 1 clear dispatch zone',3,joint_targets={0:HOME})
    w.phase('Verify assembly without robot support',1,gates=[inserted(),{'type':'released','object':'insert'},{'type':'released','object':'stand'}])
    # Pinching the thin base at its front edge tips the tall assembly. Grasp
    # the existing socket stem from the west instead; no added grasp fixture.
    sp,sr=w.pose('stand');point=sp+sr@np.array([0,.045,-.025])
    side_rotation=np.array([[0,0,1],[0,1,0],[-1,0,0.]])
    w.phase('Arm 4 approach stand from side',3,{3:(point+[-.12,0,0],side_rotation)})
    w.phase('Arm 4 engage socket stem',3,{3:(point,side_rotation)},touch={3:'stand'})
    w.phase('Arm 4 grip assembled stand',1.3,grippers={3:0},touch={3:'stand'},gates=[{'type':'grasp','object':'stand','arm':3}])
    w.holds[3]='stand'
    w.phase('Arm 4 lift assembled stand',3,{3:(point+[0,0,.06],side_rotation)},gates=[inserted()])
    w.phase('Verify seated assembly during lift',.6,gates=[inserted()])
    w.place(3,'stand',[-.30,-.06],.22,'assembly_output')
    # Clear the overhanging L before rotating the wrist. A direct joint-space
    # HOME sweeps through the assembly even when the starting TCP is clear.
    for name,delta in [('Arm 4 withdraw below overhang',[-.06,0,-.08]),
                       ('Arm 4 clear beside output',[0,-.18,0]),
                       ('Arm 4 rise outside assembly',[0,0,.22])]:
        w.phase(name,2,{3:(w.tcp(3)+delta,w.rot(3))},gates=[inserted()])
    w.phase('Arm 4 clear completed assembly',3,joint_targets={3:HOME})
    w.phase('Assembly complete - all arms home',1,gates=[inserted(),{'type':'support','object':'stand','body':'assembly_output'},
      {'type':'released','object':'stand'},{'type':'released','object':'insert'},{'type':'home'}])
    w.export()


def failure_snapshot(w,error):
    # Read raw contacts: calling w.grasp here would re-enter the penetration
    # guard and lose the diagnostic precisely when that guard caused failure.
    contacts=base.Workcell.contacts(w);grips=[]
    for object,arm in [('stand',0),('insert',1),('insert',2),('stand',3)]:
        id=w.m.body(object).id;touch={a if b==id else b for a,b,c in contacts if id in [a,b]}
        grips.append({'bilateral':w.fingers[arm]<=touch,'fingerContacts':len(w.fingers[arm]&touch),
          'aperture':sum(float(w.d.joint(f'r{arm}_finger_joint{j}').qpos[0]) for j in [1,2])})
    return {'error':str(error),'history':w.history,
          'phase':w.phases[-1]['name'] if w.phases else 'initial','tcp':[w.tcp(a).tolist() for a in range(4)],
          'actualJoints':[w.d.qpos[qa].tolist() for qa in w.qa],'grips':grips,
          'qpos':w.d.qpos.tolist(),'jointTargets':w.q.tolist(),'gripperTargets':w.grip.tolist(),'holds':w.holds,
          'objects':{o:w.pose(o)[0].tolist() for o in ['stand','insert']},
          'contacts':[{'a':w.m.body(a).name,'b':w.m.body(b).name,'distance':float(c.dist)} for a,b,c in contacts]}


if __name__=='__main__':
    w=InsertionWorkcell()
    try:run(w)
    except Exception as error:
        Path('artifacts/reports/insertion-native-failure.json').write_text(json.dumps(failure_snapshot(w,error),indent=2)+'\n')
        raise
