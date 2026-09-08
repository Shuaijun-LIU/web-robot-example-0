#!/usr/bin/env python3
"""Build and dynamically verify actuator-only Demo2 motion (CPU, no rendering).

Only the planning MjData is written for IK. Live free-body state is never set
after scene initialization. Export is conditional on physical acceptance.
"""
import argparse
import json
from pathlib import Path
import mujoco
import numpy as np
import trimesh

HOME = np.array([1.570796, -.785398, 0, -2.356194, 0, 1.570796, .785398])


def rotation_error(current, target):
    quat = np.empty(4)
    mujoco.mju_mat2Quat(quat, (target @ current.T).ravel())
    velocity = np.empty(3)
    mujoco.mju_quat2Vel(velocity, quat, 1.)
    return velocity


class Transfer:
    def __init__(self, root):
        self.model = m = mujoco.MjModel.from_xml_path(str(root / 'public/assets/franka-egg-sorting/scene.xml'))
        self.data = d = mujoco.MjData(m)
        self.planner = mujoco.MjData(m)
        for arm in range(4):
            d.ctrl[arm*8:arm*8+8] = [*HOME, 255]
            for j in range(7):
                d.joint(f'r{arm}_joint{j+1}').qpos[0] = HOME[j]
        self.joints = [m.joint(f'r0_joint{i+1}').id for i in range(7)]
        self.qa = m.jnt_qposadr[self.joints]
        self.da = m.jnt_dofadr[self.joints]
        self.site = m.site('r0_tcp').id
        self.egg = m.body('egg_0').id
        self.fingers = {m.body('r0_left_finger').id, m.body('r0_right_finger').id}
        self.tray = m.body('output_tray_0').id
        self.sources = {m.body(f'source_insert_{i}').id for i in range(4)}
        self.phases = []
        self.max_forbidden = 0.
        self.max_grip_penetration = 0.
        self.worst_contact = None
        self.contact_frames = 0
        self.samples = []
        self.max_surface_gap = 0.
        self.max_neighbor_drift = 0.
        self.max_jaw_normal_force = 0.
        self.longest_grip_loss = 0.
        self.grip_loss_start = None
        self.active_gate = None
        self.gate_failures = []
        self.q = HOME.copy()
        self.grip = 255.
        self.step(1.)
        self.start_egg = d.xpos[self.egg].copy()
        self.neighbors = d.xpos[[m.body(f'egg_{i}').id for i in range(1,16)]].copy()

    def solve(self, position, rotation, seed):
        m,p = self.model,self.planner
        p.qpos[:] = self.data.qpos
        q = np.array(seed).copy()
        jp,jr = np.zeros((3,m.nv)),np.zeros((3,m.nv))
        for _ in range(200):
            p.qpos[self.qa] = q
            mujoco.mj_kinematics(m,p); mujoco.mj_comPos(m,p)
            e = np.r_[np.array(position)-p.site_xpos[self.site], .25*rotation_error(p.site_xmat[self.site].reshape(3,3),rotation)]
            if np.linalg.norm(e) < .00005:
                return q
            mujoco.mj_jacSite(m,p,jp,jr,self.site)
            J = np.vstack([jp[:,self.da], .25*jr[:,self.da]])
            delta = J.T @ np.linalg.solve(J@J.T + .00001*np.eye(6),e)
            q += delta * min(1., .1/max(np.linalg.norm(delta),1e-9))
            q = np.clip(q, m.jnt_range[self.joints,0]+.005, m.jnt_range[self.joints,1]-.005)
        raise RuntimeError(f'IK failed at {position}: residual {np.linalg.norm(e):.5f}, joints {q.tolist()}')

    def contacts(self):
        return [(self.model.geom_bodyid[c.geom1],self.model.geom_bodyid[c.geom2],c) for c in self.data.contact]

    def bilateral(self):
        return self.fingers <= {a if b==self.egg else b for a,b,c in self.contacts() if self.egg in (a,b) and c.dist<=0}

    def audit_visible_contacts(self):
        m,d=self.model,self.data
        grouped={}
        for a,b,c in self.contacts():
            if self.egg not in (a,b):continue
            for geom,body in [(c.geom1,a),(c.geom2,b)]:
                if body not in self.fingers:continue
                grouped.setdefault(geom,[]).append(c.pos.copy())
        for geom,points in grouped.items():
            mesh=m.geom_dataid[geom]
            vertices=m.mesh_vert[m.mesh_vertadr[mesh]:m.mesh_vertadr[mesh]+m.mesh_vertnum[mesh]]
            faces=m.mesh_face[m.mesh_faceadr[mesh]:m.mesh_faceadr[mesh]+m.mesh_facenum[mesh]]
            surface=trimesh.Trimesh(vertices,faces,process=False)
            local=(np.array(points)-d.geom_xpos[geom])@d.geom_xmat[geom].reshape(3,3)
            distances=trimesh.proximity.closest_point_naive(surface,local)[1]
            self.max_surface_gap=max(self.max_surface_gap,float(max(distances)))

    def audit(self):
        m = self.model
        if hasattr(self,'start_egg'):
            neighbors=self.data.xpos[[m.body(f'egg_{i}').id for i in range(1,16)]]
            self.max_neighbor_drift=max(self.max_neighbor_drift,float(np.max(np.linalg.norm(neighbors-self.neighbors,axis=1))))
        lift=float(self.data.xpos[self.egg,2]-self.start_egg[2]) if hasattr(self,'start_egg') else 0.
        index=len(self.phases)-1
        allowed_supports=self.sources if index<=3 and lift<.02 else {self.tray} if index>=6 else set()
        jaw_forces={finger:0. for finger in self.fingers}
        for contact_index,(a,b,c) in enumerate(self.contacts()):
            names = [m.body(a).name,m.body(b).name]
            robot = [name.startswith('r') and name[1:2].isdigit() for name in names]
            allowed = self.egg in (a,b) and bool(self.fingers & {a,b})
            if allowed:
                self.max_grip_penetration=max(self.max_grip_penetration,-float(c.dist))
                force=np.empty(6)
                mujoco.mj_contactForce(m,self.data,contact_index,force)
                jaw_forces[next(iter(self.fingers & {a,b}))]+=max(0.,float(force[0]))
            forbidden=any(robot) and not allowed
            if self.egg in (a,b) and not allowed:
                forbidden=(b if a==self.egg else a) not in allowed_supports
            if forbidden and c.dist < -self.max_forbidden:
                self.max_forbidden = -float(c.dist)
                self.worst_contact = names
        self.max_jaw_normal_force=max(self.max_jaw_normal_force,max(jaw_forces.values()))
        if self.fingers <= {a if b==self.egg else b for a,b,c in self.contacts() if self.egg in (a,b) and c.dist<=0}:
            self.contact_frames += 1
        if self.active_gate in ('carried','supported') and not self.bilateral():
            if self.grip_loss_start is None:self.grip_loss_start=self.data.time
            self.longest_grip_loss=max(self.longest_grip_loss,float(self.data.time-self.grip_loss_start))
        else:self.grip_loss_start=None

    def step(self, seconds, callback=None):
        m,d = self.model,self.data
        ticks = int(round(seconds/m.opt.timestep))
        for tick in range(ticks):
            if callback: callback((tick+1)/ticks)
            # Ordinary actuator-side gravity feed-forward, not joint freezing.
            d.ctrl[:7] = self.q + d.qfrc_bias[self.da] / m.actuator_gainprm[:7,0]
            d.ctrl[7] = self.grip
            mujoco.mj_step(m,d)
            if tick%5 == 0: self.audit()

    def phase(self, name, target, duration, grip=None, gate=None, path=None):
        start = self.q.copy(); start_grip = self.grip
        end_grip = self.grip if grip is None else grip
        target = np.array(target)
        path=np.array([start,target] if path is None else path)
        self.phases.append({'name':name,'duration':duration,'joints':target.tolist(),'path':path.tolist(),'gripper':end_grip,'gate':gate})
        self.active_gate=gate
        def update(t):
            s = t*t*t*(10+t*(-15+6*t))
            index=min(int(s*(len(path)-1)),len(path)-2)
            f=s*(len(path)-1)-index
            self.q = path[index]+(path[index+1]-path[index])*f
            self.grip = start_grip+(end_grip-start_grip)*s
        self.step(duration,update)
        self.audit_visible_contacts()
        bilateral=self.bilateral()
        supported=any(self.egg in (a,b) and self.tray in (a,b) for a,b,_ in self.contacts())
        any_finger=any(self.egg in (a,b) and bool(self.fingers & {a,b}) for a,b,_ in self.contacts())
        if gate in ('bilateral','carried','supported') and not bilateral:self.gate_failures.append([name,'missing-two-sided-contact'])
        aperture=sum(float(self.data.joint(f'r0_finger_joint{i}').qpos[0]) for i in [1,2])
        if gate in ('bilateral','carried','supported'):
            if not .028<=aperture<=.070:self.gate_failures.append([name,'invalid-grip-aperture'])
            if np.linalg.norm(self.data.xpos[self.egg]-self.data.site_xpos[self.site])>.065:self.gate_failures.append([name,'egg-not-carried'])
        if gate=='carried' and self.data.xpos[self.egg,2]-self.start_egg[2]<.060:self.gate_failures.append([name,'insufficient-lift'])
        if gate in ('supported','released','seated') and not supported:self.gate_failures.append([name,'missing-tray-support'])
        if gate in ('released','seated') and any_finger:self.gate_failures.append([name,'finger-not-released'])
        if gate=='seated' and np.linalg.norm(self.data.joint('egg_0_free').qvel[:3])>.008:self.gate_failures.append([name,'egg-not-settled'])
        sample={'phase':name,'egg':self.data.xpos[self.egg].tolist(),'tcp':self.data.site_xpos[self.site].tolist(),
                'eggQuaternion':self.data.xquat[self.egg].tolist(),
                'jaws':[float(self.data.joint(f'r0_finger_joint{i}').qpos[0]) for i in [1,2]],
                'contacts':[[self.model.body(a).name,self.model.body(b).name,round(float(c.dist),6)] for a,b,c in self.contacts() if self.egg in (a,b)],
                'worstPenetration':self.max_forbidden,'worstPair':self.worst_contact}
        self.samples.append(sample)
        print(json.dumps(sample),flush=True)

    def move(self, name, position, rotation, duration, grip=None, gate=None):
        p,m=self.planner,self.model
        p.qpos[:]=self.data.qpos
        p.qpos[self.qa]=self.q
        mujoco.mj_kinematics(m,p)
        start=p.site_xpos[self.site].copy()
        qa,qb=np.empty(4),np.empty(4)
        mujoco.mju_mat2Quat(qa,p.site_xmat[self.site])
        mujoco.mju_mat2Quat(qb,rotation.ravel())
        if qa@qb<0:qb=-qb
        count=max(12,int(np.linalg.norm(np.array(position)-start)/.012))
        path=[self.q.copy()]
        for i in range(1,count+1):
            f=i/count
            quat=(1-f)*qa+f*qb;quat/=np.linalg.norm(quat)
            R=np.empty(9);mujoco.mju_quat2Mat(R,quat)
            path.append(self.solve(start+(np.array(position)-start)*f,R.reshape(3,3),path[-1]))
        self.phase(name,path[-1],duration,grip,gate,path)


def build_and_verify(root):
    t=Transfer(root)
    c=2**-.5
    # Equivalent pinch line with the opposite wrist yaw: leaves joint 7 room
    # for the later upright correction instead of ending on its upper limit.
    down=np.array([[-c,-c,0],[-c,c,0],[0,0,-1.]])
    egg=t.start_egg.copy()
    grasp=np.array([*egg[:2],.164])
    t.move('Approach above ivory egg',[*egg[:2],.34],down,3.,grip=160)
    t.move('Lower open fingers',grasp,down,3.)
    t.phase('Close on egg',t.q,2.,grip=0,gate='bilateral')
    bilateral_before_lift=t.bilateral()
    t.move('Lift clear of box',grasp+[0,0,.13],down,3.,gate='carried')
    lift=float(t.data.xpos[t.egg,2]-egg[2])
    bilateral=t.contact_frames>50
    # Correct the measured carried egg orientation in free space, through wrist
    # motion only. The egg itself is never assigned a target pose.
    offset=t.data.xpos[t.egg]-t.data.site_xpos[t.site]
    cell=np.array([-.442,-.442,.1463])
    t.move('Transfer over tray',cell-offset+[0,0,.16],down,4.,gate='carried')
    axis=t.data.xmat[t.egg].reshape(3,3)[:,2]
    v=np.cross(axis,[0,0,1.])
    skew=np.array([[0,-v[2],v[1]],[v[2],0,-v[0]],[-v[1],v[0],0]])
    align=np.eye(3)+skew+skew@skew/(1+axis[2])
    correction=align @ t.data.site_xmat[t.site].reshape(3,3)
    t.move('Align egg upright in free space',t.data.site_xpos[t.site].copy(),correction,3.,gate='carried')
    offset=t.data.xpos[t.egg]-t.data.site_xpos[t.site]
    t.move('Lower into egg tray',cell-offset,correction,4.,gate='supported')
    # Open to 51 mm at the tray (42 mm egg + 9 mm clearance), then fully
    # open during withdrawal. Opening 63 mm here sweeps a tilted tip into the rim.
    t.phase('Release egg',t.q,2.,grip=130,gate='released')
    released_aperture=sum(float(t.data.joint(f'r0_finger_joint{i}').qpos[0]) for i in [1,2])
    released_contact=any(t.egg in (a,b) and bool(t.fingers & {a,b}) for a,b,_ in t.contacts())
    t.move('Withdraw vertically',cell-offset+[0,0,.16],correction,3.,grip=255)
    t.phase('Return to hover',HOME,4.)
    before=t.data.xpos[t.egg].copy()
    t.phase('Verify stable placement',t.q,3.,gate='seated')
    supported=any(t.egg in (a,b) and t.tray in (a,b) for a,b,_ in t.contacts())
    drift=float(np.linalg.norm(t.data.xpos[t.egg]-before))
    error=float(np.linalg.norm(t.data.xpos[t.egg,:2]-cell[:2]))
    neighbors=t.data.xpos[[t.model.body(f'egg_{i}').id for i in range(1,16)]]
    neighbor_drift=t.max_neighbor_drift
    tilt=float(np.degrees(np.arccos(np.clip(t.data.xmat[t.egg].reshape(3,3)[2,2],-1,1))))
    metrics={'liftMeters':lift,'bilateralGrasp':bilateral,'maxForbiddenPenetrationMeters':t.max_forbidden,'maxGripPenetrationMeters':t.max_grip_penetration,
             'bilateralBeforeLift':bilateral_before_lift,'maxNeighborDisplacementMeters':neighbor_drift,
             'finalTiltDegrees':tilt,
             'releaseApertureMeters':released_aperture,
             'fingerContactAfterRelease':released_contact,
             'gateFailures':t.gate_failures,'longestGripLossSeconds':t.longest_grip_loss,
             'maxSingleJawNormalForceN':t.max_jaw_normal_force,
             'maxVisibleFingerContactGapMeters':t.max_surface_gap,
             'worstContact':t.worst_contact,'finalCellErrorMeters':error,'supportedAfterRelease':supported,'postReleaseDriftMeters':drift}
    success=lift>.06 and bilateral and bilateral_before_lift and t.max_forbidden<.0001 and t.max_grip_penetration<.001 and neighbor_drift<.002 and error<.012 and supported and drift<.002 and tilt<20 and released_aperture>.05 and not released_contact and t.max_surface_gap<.001 and not t.gate_failures and t.longest_grip_loss<=.12
    return {'schemaVersion':1,'nativeEngine':mujoco.__version__,'egg':'egg_0','arm':0,'initialJoints':HOME.tolist(),'initialGripper':255,
            'cell':cell.tolist(),'initialEggPosition':egg.tolist(),'initialEggPositions':[egg.tolist(),*t.neighbors.tolist()],
            'gravityFeedforward':True,'phases':t.phases,'metrics':metrics,'samples':t.samples,'success':success}


if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('--export',action='store_true')
    args=parser.parse_args()
    root=Path(__file__).resolve().parents[1]
    result=build_and_verify(root)
    print(json.dumps(result['metrics'],indent=2),flush=True)
    report=root/'artifacts/reports/demo2-first-egg-native.json'
    report.parent.mkdir(parents=True,exist_ok=True)
    report.write_text(json.dumps(result,indent=2)+'\n')
    if args.export:
        if not result['success']: raise SystemExit('Refusing to export a failed physical transfer')
        (root/'public/assets/franka-egg-sorting/first-egg-motion.json').write_text(json.dumps(result,indent=2)+'\n')
