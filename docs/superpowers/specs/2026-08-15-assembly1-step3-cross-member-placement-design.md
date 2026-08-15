# Assembly1 Step 3 Cross-Member Placement Design

## Goal

Continue from the physically verified Step 2 grasp state and use Arms 3 and 4 to lift the cross-member from its tray, transfer it above the assembly frame, lower it until all four installation holes align with the frame receivers, and hold it there. Arm 1 keeps stabilizing the frame and Arm 2 keeps holding the torque driver.

Step 3 ends with the cross-member aligned and still held by both transport arms. It does not release either arm, reposition Arm 4, insert fasteners, or run the torque driver.

## Approved Four-Arm Choreography

- Arm 1 holds the measured Step 2 frame-clamp pose and gripper command. Its frame station is offset to world `x=0.18 m`, keeping the center installation corridor clear.
- Arm 2 holds the measured Step 2 torque-driver pose and gripper command.
- Arms 3 and 4 retain their Step 2 cross-member clamps and move synchronously through the same phase clock.
- The cross-member starts at body pose `[-0.49, 0.44, 0.20]` and ends at the declared installation pose `[0, 0, 0.278]`. Its `0.018 m` half-height therefore puts its bottom face at the frame rail top (`z=0.260 m`) instead of inside the rail.
- The initial grasp points are separated by `0.24 m` in world Y; transport waypoints use the measured retained span of `0.255 m`. The final feed-forward compensated TCP targets are `[0.006, 0.1275, 0.278]` and `[0.002, -0.1275, 0.278]`.

## Motion Phases

1. `grasp-check`: require live bilateral contacts for Arm 1/frame, Arm 2/tool, and Arms 3/4/cross-member before motion.
2. `lift`: Arms 3/4 raise their TCPs from `z=0.20` to the safe transport height `z=0.38` while preserving both clamp commands.
3. `lift-settle`: hold the lifted waypoint and confirm both cross-member grasps remain bilateral.
4. `transfer-a`: translate both grasp points through the first horizontal segment at `z=0.38`.
5. `transfer-b`: use a checked Cartesian midpoint at `z=0.37`, then reach the center hover at `z=0.34`.
6. `hover-settle`: hold above the frame and confirm transport contacts and frame stability.
7. `aligned-descent`: lower both TCPs through a `z=0.295` midpoint to the compensated final targets at `z=0.278`.
8. `alignment-verification`: require all four cross-member hole sites to be within `0.008 m` of their paired frame receiver sites, cross-member orientation within `5°` of its Step 2 orientation, and frame translation within `0.008 m` of its Step 2 pose.
9. `aligned-hold`: maintain the two cross-member grasps and the two stationary-arm holds for `1 s`, then enter `complete` without releasing any gripper.

The transfer uses checked-in Panda joint targets generated from the current Menagerie MJCF and the existing selected-IK solver. Joint-space interpolation is smoothstep-clamped, and both transport arms consume one shared phase progress value.

The cross-member has physical stop ribs and top caps around both grasp stations. These features improve contact retention but do not attach the part to either gripper. Four diagnostic hole sites are defined at local `z=-0.003 m`; at the installed body pose they coincide with the frame receiver sites at world `z=0.275 m` while the visible fastening plates remain above the frame.

## Physical Integrity

Step 3 may write only the four Panda actuator blocks in `data.ctrl`. It must not write any task-object `qpos` or `qvel`, apply task-object forces, create an equality weld, use magnetic/proximity attachment, or make the cross-member follow a scripted body pose. All cross-member motion must arise from MuJoCo finger contacts.

Arm 1 and Arm 2 targets are immutable snapshots captured at Step 3 entry. On any failure, the controller freezes all four arms at freshly measured joint positions while retaining the current gripper commands; it does not open a gripper while an object may be unsupported.

## Preconditions and Failure Handling

The Step 3 button is enabled only when Step 2 is complete. Planning resolves every required arm joint, finger body, actuator, cross-member body, frame body, four hole sites, and four receiver sites before taking actuator ownership.

Planning or runtime enters `error` for missing resources, invalid Step 2 state, non-finite simulator values, joint-limit violations, lost bilateral cross-member contact, lost frame/tool hold, forbidden transport contact, excessive frame drift, excessive cross-member rotation, or alignment timeout. The UI reports the phase, arm when applicable, code, and detail, and Reset is required before retry.

## UI and Diagnostics

The Assembly1 action panel gains a third step below Step 2. Its phase text distinguishes grasp checking, lifting, transfer, hover alignment, descent, alignment verification, and stable hold. Browser diagnostics expose Step 3 phase, cross-member pose, frame drift, four hole-to-receiver distances, bilateral-contact verdicts, and actuator targets.

Manual IK, gizmo dragging, keyboard arm control, and drag interaction remain disabled while any assembly automation phase owns the actuators.

All per-frame MuJoCo contact reads consume and delete the temporary embind contact handles and vector wrapper. This prevents the previous 2 GiB WASM heap exhaustion that blanked the rendered scene during long verification runs.

## Acceptance

- Step 3 cannot start before Step 2 completes and cannot start twice.
- Arm 1 and Arm 2 do not execute a new trajectory; their captured entry targets remain unchanged.
- Arms 3 and 4 move on the same phase clock and retain their Step 2 gripper commands.
- The cross-member rises clear of the tray before horizontal translation.
- Cross-member motion is contact-driven; controller and domain sources contain no task-object position/velocity write or attachment shortcut.
- At completion, all four hole-to-receiver distances are at most `0.008 m`, frame drift is at most `0.008 m`, cross-member rotation is at most `5°`, and all four grasps remain valid for the final `1 s` hold.
- The production scene stays `ready`, all sampled controls/poses are finite, Reset restores all three steps to idle, and a final screenshot records the aligned held state.
