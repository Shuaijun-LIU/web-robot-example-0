"""Physical four-Panda pot loading program.

The caller supplies the shared ``Workcell``.  This module only commands robot
actuators; free-object state remains entirely under MuJoCo dynamics.
"""
import numpy as np


HOME = np.array([1.570796, -.785398, 0., -2.356194, 0., 1.570796, .785398])
POT_ROTATIONS = {
    0: np.diag([-1., 1., -1.]),
    2: np.diag([1., -1., -1.]),
}


def top_rotation(yaw=0.):
    c, s = np.cos(yaw), np.sin(yaw)
    return np.array([[c, s, 0.], [s, -c, 0.], [0., 0., -1.]])


def _inside_gate(food):
    return {
        'type': 'inside', 'object': food, 'container': 'cooking_pot',
        'radius': .075, 'minZ': .01, 'maxZ': .1,
    }


def _drop_in_pot(w, arm, food, local_release):
    food_position, food_rotation = w.pose(food)
    w.move_object(
        f'Arm {arm + 1} raise {food} to transfer clearance', arm, food,
        food_position + [0., 0., .12], food_rotation, duration=2.,
    )
    pot_position, pot_rotation = w.pose('cooking_pot')
    high_position = pot_position + pot_rotation @ np.asarray(local_release)
    w.move_object(
        f'Arm {arm + 1} transport {food} to high pot edge', arm, food,
        high_position, food_rotation, duration=5.,
    )
    carried_pot = {a: o for a, o in w.holds.items() if a != arm}
    w.phase(
        f'Arm {arm + 1} drop {food} into pot', 4.,
        grippers={arm: 255}, carry=carried_pot,
        gates=[_inside_gate(food)],
    )
    del w.holds[arm]
    w.phase(
        f'Arm {arm + 1} clear after dropping {food}', 2.5,
        joint_targets={arm: HOME}, touch={arm: food},
    )


def _lower_loaded_pot(w):
    position, rotation = w.pose('cooking_pot')
    target_position = np.array([0., 0., .118])
    targets = {}
    for arm in (0, 2):
        local_point = rotation.T @ (w.tcp(arm) - position)
        local_rotation = rotation.T @ w.rot(arm)
        targets[arm] = (
            target_position + rotation @ local_point,
            rotation @ local_rotation,
        )
    gates = [
        _inside_gate('carrot'), _inside_gate('tomato'),
        {'type': 'support', 'object': 'cooking_pot', 'body': 'pot_pad'},
    ]
    w.phase('Lower loaded pot onto pad', 3., targets=targets, gates=gates)

    w.phase(
        'Loosen both pot handles', 1.5,
        grippers={0: 130, 2: 130}, carry={}, gates=gates,
    )
    outside = {
        0: ([-.0035, -.1280, .2155], POT_ROTATIONS[0]),
        2: ([-.0035, .1280, .2155], POT_ROTATIONS[2]),
    }
    w.phase(
        'Withdraw fingers radially from handles', 1.5, targets=outside,
        touch={0: 'cooking_pot', 2: 'cooking_pot'}, carry={}, gates=gates,
    )
    retreat = {
        arm: (np.asarray(pose[0]) + [0., 0., .12], pose[1])
        for arm, pose in outside.items()
    }
    released = gates + [{'type': 'released', 'object': 'cooking_pot'}]
    w.phase(
        'Lift fingers clear of handle loops', 2.5, targets=retreat,
        touch={0: 'cooking_pot', 2: 'cooking_pot'}, carry={}, gates=released,
    )
    w.phase(
        'Open clear grippers fully', 1.,
        grippers={0: 255, 2: 255}, carry={}, gates=released,
    )
    del w.holds[0]
    del w.holds[2]


def run_pot(w):
    """Load carrot and tomato into a cooperatively held pot and reset."""
    insert = {
        0: ([-.0035, -.1280, .2155], POT_ROTATIONS[0]),
        2: ([-.0035, .1280, .2155], POT_ROTATIONS[2]),
    }
    above = {
        arm: (np.asarray(pose[0]) + [0., 0., .13], pose[1])
        for arm, pose in insert.items()
    }
    touch_pot = {0: 'cooking_pot', 2: 'cooking_pot'}
    w.phase('Approach pot handles from above', 3., above,
            grippers={0: 130, 2: 130}, touch=touch_pot)
    w.phase('Insert fingers around pot handles', 2.5, insert,
            touch=touch_pot)

    grasp = {
        0: ([-.0035, -.1199, .2155], POT_ROTATIONS[0]),
        2: ([-.0035, .1199, .2155], POT_ROTATIONS[2]),
    }
    w.phase('Recenter fingers around handle bars', 1.5, grasp,
            touch=touch_pot)
    w.phase(
        'Close both pot handles', 2., grippers={0: 0, 2: 0},
        touch=touch_pot,
        gates=[
            {'type': 'grasp', 'object': 'cooking_pot', 'arm': 0},
            {'type': 'grasp', 'object': 'cooking_pot', 'arm': 2},
        ],
    )
    w.holds[0] = 'cooking_pot'
    w.holds[2] = 'cooking_pot'
    lifted = {
        arm: (np.asarray(pose[0]) + [0., 0., .035], pose[1])
        for arm, pose in grasp.items()
    }
    w.phase(
        'Lift pot for loading', 3., lifted,
        gates=[
            {'type': 'grasp', 'object': 'cooking_pot', 'arm': 0},
            {'type': 'grasp', 'object': 'cooking_pot', 'arm': 2},
            {'type': 'height', 'object': 'cooking_pot', 'minZ': .145},
        ],
    )

    carrot_position, carrot_rotation = w.pose('carrot')
    carrot_grasp = carrot_position + carrot_rotation @ np.array([0., 0., .0185])
    carrot_grasp[2] = .140
    w.pick(1, 'carrot', carrot_grasp, top_rotation(np.pi / 2))
    _drop_in_pot(w, 1, 'carrot', [-.018, -.060, .200])

    tomato_position, tomato_rotation = w.pose('tomato')
    tomato_grasp = tomato_position + tomato_rotation @ np.array([0., 0., .03])
    w.pick(3, 'tomato', tomato_grasp, top_rotation(np.pi / 2))
    _drop_in_pot(w, 3, 'tomato', [0., .050, .200])

    _lower_loaded_pot(w)
    final_gates = [
        _inside_gate('carrot'), _inside_gate('tomato'),
        {'type': 'support', 'object': 'cooking_pot', 'body': 'pot_pad'},
        {'type': 'released', 'object': 'cooking_pot'},
        {'type': 'home'},
    ]
    w.phase(
        'Pot task complete - all arms home', 3.,
        joint_targets={arm: HOME for arm in range(4)},
        gates=final_gates,
    )
