#!/usr/bin/env python3
"""Separate actuator-only post-release correction trial; baseline stays intact."""
import argparse
import importlib.util
import json
from pathlib import Path
import mujoco
import numpy as np

spec=importlib.util.spec_from_file_location('egg_transfer',Path(__file__).with_name('solve-egg-transfer.py'))
base=importlib.util.module_from_spec(spec);spec.loader.exec_module(base)

def pose(t):
    p=t.planner;p.qpos[:]=t.data.qpos;p.qpos[t.qa]=t.q
    mujoco.mj_kinematics(t.model,p)
    return p.site_xpos[t.site].copy(),p.site_xmat[t.site].reshape(3,3).copy()

def tilt(t):
    return float(np.degrees(np.arccos(np.clip(t.data.xmat[t.egg].reshape(3,3)[2,2],-1,1))))

def released(t):
    pairs=[{a,b} for a,b,c in t.contacts() if c.dist<=0]
    supported={t.egg,t.tray} in pairs
    touching=any(t.egg in pair and bool(t.fingers&pair) for pair in pairs)
    return supported,not touching

def build_and_verify(root):
    baseline=json.loads((root/'public/assets/franka-egg-sorting/first-egg-motion.json').read_text())
    t=base.Transfer(root)
    for phase in baseline['phases'][:7]:
        t.phase(phase['name'],phase['joints'],phase['duration'],phase['gripper'],phase['gate'],phase['path'])
    tcp,R=pose(t)
    # A physically executed early wrist withdrawal creates the test error.
    # This deliberately differs from the accepted baseline's stationary release.
    t.move('Release with an early wrist withdrawal',tcp+[0,0,.025],R,2.,grip=160,gate='released')
    t.move('Clear the tilted egg',tcp+[0,0,.16],R,3.,grip=255)
    t.phase('Inspect released egg tilt',t.q,2.,gate='released')
    t.phases[-1]['gate']='tilted'
    bad_position=t.data.xpos[t.egg].copy();bad_tilt=tilt(t)
    supported,free=released(t)
    initial_released=supported and free
    bad_quat=t.data.xquat[t.egg].copy()
    c=2**-.5
    down=np.array([[-c,-c,0],[-c,c,0],[0,0,-1.]])
    grasp=bad_position+[0,0,.025]
    t.move('Approach tilted egg',grasp+[0,0,.14],down,3.,grip=160)
    t.move('Lower for a second grasp',grasp,down,3.)
    t.phase('Regrasp both sides',t.q,2.,grip=0,gate='bilateral')
    regrasp=t.bilateral()
    before_lift=t.data.xpos[t.egg].copy()
    t.move('Lift egg out of its pocket',grasp+[0,0,.10],down,3.,gate='carried')
    relift=float(t.data.xpos[t.egg,2]-before_lift[2])
    axis=t.data.xmat[t.egg].reshape(3,3)[:,2]
    v=np.cross(axis,[0,0,1.]);skew=np.array([[0,-v[2],v[1]],[v[2],0,-v[0]],[-v[1],v[0],0]])
    correction=(np.eye(3)+skew+skew@skew/(1+axis[2]))@t.data.site_xmat[t.site].reshape(3,3)
    t.move('Correct the regrasped egg upright',pose(t)[0],correction,3.,gate='carried')
    offset=t.data.xpos[t.egg]-t.data.site_xpos[t.site]
    cell=np.array(baseline['cell'])
    t.move('Reseat the corrected egg',cell-offset,correction,4.,gate='supported')
    t.phase('Release the corrected egg',t.q,2.,grip=130,gate='released')
    t.move('Withdraw after correction',cell-offset+[0,0,.16],correction,3.,grip=255)
    t.phase('Return after correction',base.HOME,4.)
    before=t.data.xpos[t.egg].copy()
    t.phase('Verify corrected placement',t.q,3.,gate='seated')
    supported,free=released(t)
    metrics={'releasedTiltDegrees':bad_tilt,'releasedBeforeCorrection':initial_released,
        'bilateralRegrasp':regrasp,'reliftMeters':relift,'finalTiltDegrees':tilt(t),
        'supportedAfterRelease':supported,'fingerContactAfterRelease':not free,
        'maxForbiddenPenetrationMeters':t.max_forbidden,'maxGripPenetrationMeters':t.max_grip_penetration,
        'maxNeighborDisplacementMeters':t.max_neighbor_drift,'maxVisibleFingerContactGapMeters':t.max_surface_gap,
        'maxSingleJawNormalForceN':t.max_jaw_normal_force,'longestGripLossSeconds':t.longest_grip_loss,
        'postReleaseDriftMeters':float(np.linalg.norm(t.data.xpos[t.egg]-before)),
        'finalCellErrorMeters':float(np.linalg.norm(t.data.xpos[t.egg,:2]-cell[:2])),
        'gateFailures':t.gate_failures,'worstContact':t.worst_contact}
    success=initial_released and bad_tilt>20 and regrasp and relift>.06 and tilt(t)<20 and bad_tilt-tilt(t)>10 and supported and free and t.max_forbidden<.0001 and t.max_grip_penetration<.001 and t.max_neighbor_drift<.002 and t.max_surface_gap<.001 and t.longest_grip_loss<=.12 and not t.gate_failures and metrics['postReleaseDriftMeters']<.002 and metrics['finalCellErrorMeters']<.012
    return {**{k:baseline[k] for k in ['egg','arm','initialJoints','initialGripper','initialEggPosition','initialEggPositions','cell','gravityFeedforward']},
        'schemaVersion':2,'program':'reseat','nativeEngine':mujoco.__version__,
        'inspection':{'phaseIndex':9,'position':bad_position.tolist(),'quaternion':bad_quat.tolist(),'tiltDegrees':bad_tilt},
        'phases':t.phases,'metrics':metrics,'samples':t.samples,'success':bool(success)}

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--export',action='store_true');args=parser.parse_args()
    root=Path(__file__).resolve().parents[1];result=build_and_verify(root)
    print(json.dumps(result['metrics'],indent=2),flush=True)
    (root/'artifacts/reports/demo2-egg-reseat-native.json').write_text(json.dumps(result,indent=2)+'\n')
    if args.export:
        if not result['success']:raise SystemExit('Refusing to export a failed correction')
        (root/'public/assets/franka-egg-sorting/egg-reseat-motion.json').write_text(json.dumps(result,indent=2)+'\n')
