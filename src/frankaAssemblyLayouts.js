import { fixedBox, FRANKA_HOME, repeatPose } from './sceneLayouts.js';

const QUARTER_TURN_DEGREES = 90;
const HALF_TURN_DEGREES = 180;
const PLATFORM_TOP = 0.1;
const RING_RADIUS = 0.9;

const TASK_STATIONS = {
  frame: [0, 0, 0.275],
  parts: [-0.56, 0.42, 0.125],
  poweredTool: [0.53, -0.42, 0.135],
  manualTool: [-0.53, -0.42, 0.13],
  hammer: [0.65, 0, 0.229],
  fasteners: [0.56, 0.42, 0.125],
  handover: [0, -0.48, 0.112],
};

export const FRANKA_ASSEMBLY_INTERFACE = {
  crossMemberTargetPose: [0, 0, 0.235],
  frameReceiverPositions: [
    [-0.04, 0.215, 0.275],
    [0.04, 0.215, 0.275],
    [-0.04, -0.215, 0.275],
    [0.04, -0.215, 0.275],
  ],
  crossMemberHoleLocalPositions: [
    [-0.04, 0.215, 0.04],
    [0.04, 0.215, 0.04],
    [-0.04, -0.215, 0.04],
    [0.04, -0.215, 0.04],
  ],
};

export function applyAssemblyTargetPose(points, targetPose) {
  return points.map((point) => point.map((value, axis) => value + targetPose[axis]));
}

function attachmentFrames() {
  return [
    { position: [0, -RING_RADIUS, PLATFORM_TOP], yaw: 0 },
    { position: [RING_RADIUS, 0, PLATFORM_TOP], yaw: QUARTER_TURN_DEGREES },
    { position: [0, RING_RADIUS, PLATFORM_TOP], yaw: HALF_TURN_DEGREES },
    { position: [-RING_RADIUS, 0, PLATFORM_TOP], yaw: -QUARTER_TURN_DEGREES },
  ].map(({ position, yaw }, index) => {
    const euler = yaw === 0 ? '' : ` euler="0 0 ${yaw}"`;
    return `<frame pos="${position.join(' ')}"${euler}><attach model="panda_model" body="link0" prefix="r${index}_"/></frame>`;
  }).join('');
}

function octagonalHandleMesh() {
  const rings = [
    [-0.105, 0.021], [-0.098, 0.0255], [-0.071, 0.0255], [-0.066, 0.022],
    [-0.058, 0.022], [-0.053, 0.0255], [-0.023, 0.0255], [-0.018, 0.022],
    [-0.010, 0.022], [-0.005, 0.0255], [0.027, 0.0255], [0.039, 0.020],
  ];
  const vertices = rings.flatMap(([x, radius]) => Array.from({ length: 8 }, (_, index) => {
    const angle = Math.PI / 8 + index * Math.PI / 4;
    return [x, radius * Math.cos(angle), radius * Math.sin(angle)];
  }));
  const faces = [];
  for (let ring = 0; ring < rings.length - 1; ring += 1) {
    for (let side = 0; side < 8; side += 1) {
      const next = (side + 1) % 8;
      const a = ring * 8 + side;
      const b = ring * 8 + next;
      const c = (ring + 1) * 8 + next;
      const d = (ring + 1) * 8 + side;
      faces.push([a, b, c], [a, c, d]);
    }
  }
  for (let side = 1; side < 7; side += 1) {
    faces.push([0, side + 1, side]);
    const end = (rings.length - 1) * 8;
    faces.push([end, end + side, end + side + 1]);
  }
  return `vertex="${vertices.flat().map((value) => value.toFixed(6)).join(' ')}" face="${faces.flat().join(' ')}"`;
}

const SHARED_WORKCELL_XML = `
    <!-- Four supports hold the movable frame at the same height as its installation pose. -->
    <body name="frame_supports">
      <geom name="frame_support_nw" type="box" pos="-.27 .18 .155" size=".055 .045 .055" rgba=".16 .18 .2 1"/>
      <geom name="frame_support_ne" type="box" pos=".27 .18 .155" size=".055 .045 .055" rgba=".16 .18 .2 1"/>
      <geom name="frame_support_sw" type="box" pos="-.27 -.18 .155" size=".055 .045 .055" rgba=".16 .18 .2 1"/>
      <geom name="frame_support_se" type="box" pos=".27 -.18 .155" size=".055 .045 .055" rgba=".16 .18 .2 1"/>
    </body>

    <!-- Paired silver flanges and dark T-slots make the aluminum extrusion readable. -->
    <body name="assembly_frame" pos="0 0 .235">
      <freejoint/>
      <geom name="frame_rail_north_outer" type="box" pos="0 .242 0" size=".34 .012 .025" rgba=".55 .57 .58 1"/>
      <geom name="frame_rail_north_inner" type="box" pos="0 .218 0" size=".34 .008 .025" rgba=".68 .69 .69 1"/>
      <geom name="frame_rail_north_slot" type="box" pos="0 .229 .026" size=".31 .006 .003" rgba=".08 .09 .1 1" contype="0" conaffinity="0"/>
      <geom name="frame_rail_south_outer" type="box" pos="0 -.242 0" size=".34 .012 .025" rgba=".55 .57 .58 1"/>
      <geom name="frame_rail_south_inner" type="box" pos="0 -.218 0" size=".34 .008 .025" rgba=".68 .69 .69 1"/>
      <geom name="frame_rail_south_slot" type="box" pos="0 -.229 .026" size=".31 .006 .003" rgba=".08 .09 .1 1" contype="0" conaffinity="0"/>
      <geom name="frame_rail_west_outer" type="box" pos="-.327 0 0" size=".013 .205 .025" rgba=".55 .57 .58 1"/>
      <geom name="frame_rail_west_inner" type="box" pos="-.303 0 0" size=".008 .205 .025" rgba=".68 .69 .69 1"/>
      <geom name="frame_rail_west_slot" type="box" pos="-.315 0 .026" size=".006 .18 .003" rgba=".08 .09 .1 1" contype="0" conaffinity="0"/>
      <geom name="frame_rail_east_outer" type="box" pos=".327 0 0" size=".013 .205 .025" rgba=".55 .57 .58 1"/>
      <geom name="frame_rail_east_inner" type="box" pos=".303 0 0" size=".008 .205 .025" rgba=".68 .69 .69 1"/>
      <geom name="frame_rail_east_slot" type="box" pos=".315 0 .026" size=".006 .18 .003" rgba=".08 .09 .1 1" contype="0" conaffinity="0"/>
      <geom name="frame_grip_west" type="box" pos="-.354 0 .006" size=".014 .08 .034" rgba=".15 .17 .18 1"/>
      <geom name="frame_grip_east" type="box" pos=".354 0 .006" size=".014 .08 .034" rgba=".15 .17 .18 1"/>
      <site name="frame_receiver_nw" pos="-.04 .215 .04" type="cylinder" size=".012 .002" rgba=".07 .08 .09 1"/>
      <site name="frame_receiver_ne" pos=".04 .215 .04" type="cylinder" size=".012 .002" rgba=".07 .08 .09 1"/>
      <site name="frame_receiver_sw" pos="-.04 -.215 .04" type="cylinder" size=".012 .002" rgba=".07 .08 .09 1"/>
      <site name="frame_receiver_se" pos=".04 -.215 .04" type="cylinder" size=".012 .002" rgba=".07 .08 .09 1"/>
    </body>

    <body name="parts_tray" pos="-.57 .44 .11">
      <geom name="parts_tray_floor" type="box" size=".25 .29 .01" rgba=".24 .3 .34 1"/>
      <geom name="parts_tray_west_wall" type="box" pos="-.245 0 .035" size=".008 .29 .035" rgba=".19 .24 .28 1"/>
      <geom name="parts_tray_east_wall" type="box" pos=".245 0 .035" size=".008 .29 .035" rgba=".19 .24 .28 1"/>
      <geom name="parts_tray_north_wall" type="box" pos="0 .285 .035" size=".25 .008 .035" rgba=".19 .24 .28 1"/>
      <geom name="parts_tray_south_wall" type="box" pos="0 -.285 .035" size=".25 .008 .035" rgba=".19 .24 .28 1"/>
    </body>

    <!-- This staged cross-member fits the four receivers at target pose (0, 0, .235). -->
    <body name="cross_member" pos="-.49 .44 .14">
      <freejoint/>
      <geom name="cross_member_flange_left" type="box" pos="-.016 0 0" size=".009 .245 .018" rgba=".56 .58 .59 1" mass=".18" friction="1.2 .2 .02"/>
      <geom name="cross_member_flange_right" type="box" pos=".016 0 0" size=".009 .245 .018" rgba=".68 .69 .69 1" mass=".18" friction="1.2 .2 .02"/>
      <geom name="cross_member_slot" type="box" pos="0 0 .019" size=".005 .205 .002" rgba=".08 .09 .1 1" contype="0" conaffinity="0"/>
      <geom name="cross_member_north_plate_outer" type="box" pos="0 .239 .04" size=".076 .006 .008" rgba=".24 .27 .29 1" mass=".015"/>
      <geom name="cross_member_north_plate_inner" type="box" pos="0 .191 .04" size=".076 .006 .008" rgba=".24 .27 .29 1" mass=".015"/>
      <geom name="cross_member_north_plate_left" type="box" pos="-.071 .215 .04" size=".005 .018 .008" rgba=".24 .27 .29 1" mass=".01"/>
      <geom name="cross_member_north_plate_center" type="box" pos="0 .215 .04" size=".005 .018 .008" rgba=".24 .27 .29 1" mass=".01"/>
      <geom name="cross_member_north_plate_right" type="box" pos=".071 .215 .04" size=".005 .018 .008" rgba=".24 .27 .29 1" mass=".01"/>
      <geom name="cross_member_south_plate_outer" type="box" pos="0 -.239 .04" size=".076 .006 .008" rgba=".24 .27 .29 1" mass=".015"/>
      <geom name="cross_member_south_plate_inner" type="box" pos="0 -.191 .04" size=".076 .006 .008" rgba=".24 .27 .29 1" mass=".015"/>
      <geom name="cross_member_south_plate_left" type="box" pos="-.071 -.215 .04" size=".005 .018 .008" rgba=".24 .27 .29 1" mass=".01"/>
      <geom name="cross_member_south_plate_center" type="box" pos="0 -.215 .04" size=".005 .018 .008" rgba=".24 .27 .29 1" mass=".01"/>
      <geom name="cross_member_south_plate_right" type="box" pos=".071 -.215 .04" size=".005 .018 .008" rgba=".24 .27 .29 1" mass=".01"/>
      <site name="cross_member_north_hole_left" pos="-.04 .215 .04" type="cylinder" size=".012 .002" rgba=".07 .08 .09 1"/>
      <site name="cross_member_north_hole_right" pos=".04 .215 .04" type="cylinder" size=".012 .002" rgba=".07 .08 .09 1"/>
      <site name="cross_member_south_hole_left" pos="-.04 -.215 .04" type="cylinder" size=".012 .002" rgba=".07 .08 .09 1"/>
      <site name="cross_member_south_hole_right" pos=".04 -.215 .04" type="cylinder" size=".012 .002" rgba=".07 .08 .09 1"/>
      <site name="cross_member_hole_nw" pos="-.04 .215 .04" size=".004" rgba=".8 .55 .18 .35"/>
      <site name="cross_member_hole_ne" pos=".04 .215 .04" size=".004" rgba=".8 .55 .18 .35"/>
      <site name="cross_member_hole_sw" pos="-.04 -.215 .04" size=".004" rgba=".8 .55 .18 .35"/>
      <site name="cross_member_hole_se" pos=".04 -.215 .04" size=".004" rgba=".8 .55 .18 .35"/>
    </body>

    <body name="mounting_plate" pos="-.72 .44 .135">
      <freejoint/>
      <geom name="mounting_plate_body" type="box" size=".065 .09 .012" rgba=".48 .5 .52 1" mass=".18" friction="1.2 .2 .02"/>
      <geom name="mounting_plate_boss" type="cylinder" pos="0 0 .02" size=".026 .008" rgba=".22 .24 .25 1" mass=".02"/>
      <site name="mounting_plate_hole" pos="0 0 .031" type="cylinder" size=".009 .002" rgba=".06 .07 .08 1"/>
    </body>

    <body name="fastener_tray" pos=".56 .42 .11">
      <geom name="fastener_tray_floor" type="box" size=".18 .18 .01" rgba=".3 .32 .34 1"/>
      <geom name="fastener_tray_west_wall" type="box" pos="-.175 0 .03" size=".008 .18 .03" rgba=".21 .23 .25 1"/>
      <geom name="fastener_tray_east_wall" type="box" pos=".175 0 .03" size=".008 .18 .03" rgba=".21 .23 .25 1"/>
      <geom name="fastener_tray_north_wall" type="box" pos="0 .175 .03" size=".18 .008 .03" rgba=".21 .23 .25 1"/>
      <geom name="fastener_tray_south_wall" type="box" pos="0 -.175 .03" size=".18 .008 .03" rgba=".21 .23 .25 1"/>
    </body>
    <body name="fastener_1" pos=".50 .36 .152"><freejoint/><geom name="fastener_1_shaft" type="cylinder" size=".007 .025" rgba=".42 .43 .44 1" mass=".012"/><geom name="fastener_1_head" type="cylinder" pos="0 0 .032" size=".015 .007" rgba=".16 .17 .18 1" mass=".006"/></body>
    <body name="fastener_2" pos=".60 .36 .152"><freejoint/><geom name="fastener_2_shaft" type="cylinder" size=".007 .025" rgba=".42 .43 .44 1" mass=".012"/><geom name="fastener_2_head" type="cylinder" pos="0 0 .032" size=".015 .007" rgba=".16 .17 .18 1" mass=".006"/></body>
    <body name="fastener_3" pos=".50 .48 .152"><freejoint/><geom name="fastener_3_shaft" type="cylinder" size=".007 .025" rgba=".42 .43 .44 1" mass=".012"/><geom name="fastener_3_head" type="cylinder" pos="0 0 .032" size=".015 .007" rgba=".16 .17 .18 1" mass=".006"/></body>
    <body name="fastener_4" pos=".60 .48 .152"><freejoint/><geom name="fastener_4_shaft" type="cylinder" size=".007 .025" rgba=".42 .43 .44 1" mass=".012"/><geom name="fastener_4_head" type="cylinder" pos="0 0 .032" size=".015 .007" rgba=".16 .17 .18 1" mass=".006"/></body>`;

const ASSEMBLY1_ASSET_XML = `<mesh name="manual_screwdriver_octagonal_handle" ${octagonalHandleMesh()}/>`;

const ASSEMBLY1_TOOL_XML = `
    <body name="manual_screwdriver" pos="-.53 -.42 .145">
      <joint name="manual_screwdriver_free" type="free" damping=".08"/>
      <geom name="manual_screwdriver_handle" type="mesh" mesh="manual_screwdriver_octagonal_handle" rgba=".48 .19 .07 1" contype="0" conaffinity="0" mass=".001"/>
      <geom name="manual_screwdriver_handle_collision" type="box" pos="-.033 0 0" size=".072 .022 .022" rgba="0 0 0 0" mass=".109" friction="1.6 .25 .03"/>
      <geom name="manual_screwdriver_collar" type="cylinder" fromto=".039 0 0 .066 0 0" size=".015" rgba=".16 .17 .18 1" mass=".02"/>
      <geom name="manual_screwdriver_shaft" type="cylinder" fromto=".066 0 0 .19 0 0" size=".006" rgba=".5 .51 .52 1" mass=".035"/>
      <geom name="manual_screwdriver_tip" type="box" pos=".199 0 0" size=".012 .004 .002" rgba=".2 .21 .22 1" mass=".005"/>
    </body>

    <body name="torque_driver" pos=".53 -.42 .166" euler="90 0 0">
      <freejoint/>
      <geom name="torque_driver_housing" type="capsule" fromto="-.045 0 .064 .055 0 .064" size=".038" rgba=".42 .22 .07 1" mass=".2" friction="1.3 .2 .02"/>
      <geom name="torque_driver_rear_cap" type="box" pos=".055 0 .064" size=".018 .035 .033" rgba=".14 .15 .16 1" mass=".03"/>
      <geom name="torque_driver_gearbox" type="cylinder" fromto="-.045 0 .064 -.082 0 .064" size=".029" rgba=".24 .25 .26 1" mass=".05"/>
      <geom name="torque_driver_selector" type="cylinder" fromto="-.082 0 .064 -.099 0 .064" size=".023" rgba=".1 .11 .12 1" mass=".02"/>
      <geom name="torque_driver_chuck" type="cylinder" fromto="-.099 0 .064 -.132 0 .064" size=".017" rgba=".18 .19 .2 1" mass=".03"/>
      <geom name="torque_driver_bit" type="cylinder" fromto="-.132 0 .064 -.196 0 .064" size=".005" rgba=".5 .51 .52 1" mass=".012"/>
      <geom name="torque_driver_grip" type="capsule" fromto=".015 0 .044 .043 0 -.043" size=".025" rgba=".12 .13 .14 1" mass=".12"/>
      <geom name="torque_driver_trigger" type="box" pos="-.006 -.027 .018" size=".015 .008 .018" euler="-18 0 0" rgba=".62 .33 .09 1" mass=".01"/>
      <geom name="torque_driver_battery" type="box" pos=".047 0 -.078" size=".054 .044 .015" rgba=".1 .11 .12 1" mass=".13"/>
      <geom name="torque_driver_battery_foot" type="box" pos=".047 0 -.095" size=".06 .048 .007" rgba=".2 .21 .22 1" mass=".03"/>
      <geom name="torque_driver_vent_left" type="box" pos=".048 -.036 .074" size=".026 .002 .003" rgba=".05 .06 .07 1" contype="0" conaffinity="0"/>
      <geom name="torque_driver_vent_right" type="box" pos=".048 .036 .074" size=".026 .002 .003" rgba=".05 .06 .07 1" contype="0" conaffinity="0"/>
      <geom name="torque_driver_vent_left_2" type="box" pos=".048 -.036 .062" size=".026 .002 .003" rgba=".05 .06 .07 1" contype="0" conaffinity="0"/>
      <geom name="torque_driver_vent_right_2" type="box" pos=".048 .036 .062" size=".026 .002 .003" rgba=".05 .06 .07 1" contype="0" conaffinity="0"/>
    </body>

    <body name="double_face_hammer" pos=".65 0 .229" euler="0 0 180">
      <freejoint/>
      <geom name="hammer_handle_core" type="box" pos="-.045 0 -.005" size=".105 .014 .014" rgba=".43 .22 .08 1" mass=".1" friction="1.4 .22 .03"/>
      <geom name="hammer_handle_grip" type="box" pos="-.083 0 -.005" size=".073 .021 .018" rgba=".11 .12 .13 1" mass=".12"/>
      <geom name="hammer_eye" type="cylinder" fromto=".048 0 0 .102 0 0" size=".021" rgba=".18 .19 .2 1" mass=".08"/>
      <geom name="hammer_cheek" type="box" pos=".075 0 0" size=".032 .03 .025" rgba=".32 .33 .34 1" mass=".18"/>
      <geom name="hammer_face_neck_a" type="cylinder" fromto=".075 -.030 0 .075 -.053 0" size=".019" rgba=".36 .37 .38 1" mass=".04"/>
      <geom name="hammer_striking_face_a" type="cylinder" fromto=".075 -.053 0 .075 -.073 0" size=".027" rgba=".5 .51 .52 1" mass=".08"/>
      <geom name="hammer_face_neck_b" type="cylinder" fromto=".075 .030 0 .075 .053 0" size=".019" rgba=".36 .37 .38 1" mass=".04"/>
      <geom name="hammer_striking_face_b" type="cylinder" fromto=".075 .053 0 .075 .073 0" size=".027" rgba=".5 .51 .52 1" mass=".08"/>
    </body>`;

const ASSEMBLY2_ASSET_XML = `
      <material name="robotwin_screwdriver_primary_material" rgba=".90 .55 .06 1" specular=".2" shininess=".18"/>
      <material name="robotwin_screwdriver_dark_material" rgba=".08 .09 .10 1" specular=".16" shininess=".12"/>
      <material name="robotwin_screwdriver_metal_material" rgba=".58 .60 .62 1" specular=".55" shininess=".42"/>
      <material name="robotwin_drill_primary_material" rgba=".34 .32 .29 1" specular=".18" shininess=".15"/>
      <material name="robotwin_drill_dark_material" rgba=".07 .08 .09 1" specular=".18" shininess=".14"/>
      <material name="robotwin_drill_metal_material" rgba=".62 .64 .65 1" specular=".58" shininess=".45"/>
      <material name="robotwin_hammer_primary_material" rgba=".84 .55 .04 1" specular=".16" shininess=".12"/>
      <material name="robotwin_hammer_dark_material" rgba=".07 .08 .09 1" specular=".16" shininess=".12"/>
      <material name="robotwin_hammer_metal_material" rgba=".60 .62 .63 1" specular=".6" shininess=".48"/>
      <mesh name="robotwin_screwdriver_primary" file="tools/robotwin-screwdriver-primary.obj" scale=".095 .095 .095"/>
      <mesh name="robotwin_screwdriver_dark" file="tools/robotwin-screwdriver-dark.obj" scale=".095 .095 .095"/>
      <mesh name="robotwin_screwdriver_metal" file="tools/robotwin-screwdriver-metal.obj" scale=".095 .095 .095"/>
      <mesh name="robotwin_drill_primary" file="tools/robotwin-drill-primary.obj" scale=".105 .105 .105"/>
      <mesh name="robotwin_drill_dark" file="tools/robotwin-drill-dark.obj" scale=".105 .105 .105"/>
      <mesh name="robotwin_drill_metal" file="tools/robotwin-drill-metal.obj" scale=".105 .105 .105"/>
      <mesh name="robotwin_hammer_primary" file="tools/robotwin-hammer-primary.obj" scale=".11 .11 .11"/>
      <mesh name="robotwin_hammer_dark" file="tools/robotwin-hammer-dark.obj" scale=".11 .11 .11"/>
      <mesh name="robotwin_hammer_metal" file="tools/robotwin-hammer-metal.obj" scale=".11 .11 .11"/>`;

const ASSEMBLY2_TOOL_XML = `
    <body name="manual_screwdriver" pos="-.53 -.42 .145">
      <freejoint/>
      <geom name="robotwin_screwdriver_primary_visual_geom" type="mesh" mesh="robotwin_screwdriver_primary" material="robotwin_screwdriver_primary_material" pos="0 0 -.012" contype="0" conaffinity="0" mass=".001"/>
      <geom name="robotwin_screwdriver_dark_visual_geom" type="mesh" mesh="robotwin_screwdriver_dark" material="robotwin_screwdriver_dark_material" pos="0 0 -.012" contype="0" conaffinity="0" mass=".001"/>
      <geom name="robotwin_screwdriver_metal_visual_geom" type="mesh" mesh="robotwin_screwdriver_metal" material="robotwin_screwdriver_metal_material" pos="0 0 -.012" contype="0" conaffinity="0" mass=".001"/>
      <geom name="robotwin_screwdriver_collision" type="capsule" fromto="-.09 0 0 .04 0 0" size=".025" rgba="0 0 0 0" mass=".1" friction="1.5 .25 .03"/>
      <geom name="robotwin_screwdriver_shaft_collision" type="capsule" fromto=".04 0 0 .19 0 0" size=".006" rgba="0 0 0 0" mass=".03"/>
    </body>
    <body name="torque_driver" pos=".53 -.42 .222">
      <freejoint/>
      <geom name="robotwin_drill_primary_visual_geom" type="mesh" mesh="robotwin_drill_primary" material="robotwin_drill_primary_material" pos="0 0 -.018" contype="0" conaffinity="0" mass=".001"/>
      <geom name="robotwin_drill_dark_visual_geom" type="mesh" mesh="robotwin_drill_dark" material="robotwin_drill_dark_material" pos="0 0 -.018" contype="0" conaffinity="0" mass=".001"/>
      <geom name="robotwin_drill_metal_visual_geom" type="mesh" mesh="robotwin_drill_metal" material="robotwin_drill_metal_material" pos="0 0 -.018" contype="0" conaffinity="0" mass=".001"/>
      <geom name="robotwin_drill_collision" type="box" pos="0 0 .045" size=".09 .04 .055" rgba="0 0 0 0" mass=".28" friction="1.3 .2 .02"/>
      <geom name="robotwin_drill_grip_collision" type="box" pos=".03 0 -.022" size=".026 .023 .045" rgba="0 0 0 0" mass=".12" friction="1.5 .25 .03"/>
      <geom name="robotwin_drill_battery_collision" type="box" pos=".04 0 -.088" size=".055 .045 .015" rgba="0 0 0 0" mass=".12"/>
    </body>
    <body name="claw_hammer" pos=".65 0 .229">
      <freejoint/>
      <geom name="robotwin_hammer_primary_visual_geom" type="mesh" mesh="robotwin_hammer_primary" material="robotwin_hammer_primary_material" pos="0 0 -.008" euler="90 0 0" contype="0" conaffinity="0" mass=".001"/>
      <geom name="robotwin_hammer_dark_visual_geom" type="mesh" mesh="robotwin_hammer_dark" material="robotwin_hammer_dark_material" pos="0 0 -.008" euler="90 0 0" contype="0" conaffinity="0" mass=".001"/>
      <geom name="robotwin_hammer_metal_visual_geom" type="mesh" mesh="robotwin_hammer_metal" material="robotwin_hammer_metal_material" pos="0 0 -.008" euler="90 0 0" contype="0" conaffinity="0" mass=".001"/>
      <geom name="robotwin_hammer_collision" type="capsule" fromto="-.14 0 -.008 .06 0 -.008" size=".02" rgba="0 0 0 0" mass=".16" friction="1.4 .22 .03"/>
      <geom name="robotwin_hammer_head_collision" type="box" pos=".075 0 0" size=".05 .03 .026" rgba="0 0 0 0" mass=".3"/>
    </body>`;

const sceneObjects = (includeTorqueDriverCradle = false) => [
  fixedBox('assembly_platform', [1.15, 1.15, .05], [0, 0, .05], [.25, .27, .29, 1]),
  fixedBox('platform_inset', [.82, .82, .006], [0, 0, .106], [.33, .35, .36, 1]),
  fixedBox('handover_pad', [.16, .11, .006], [0, -.48, .112], [.24, .31, .36, 1]),
  fixedBox('tool_mat_powered', [.2, .13, .006], [.53, -.42, .112], [.31, .27, .21, 1]),
  fixedBox('tool_mat_manual', [.2, .13, .006], [-.53, -.42, .112], [.31, .27, .21, 1]),
  fixedBox('tool_mat_hammer', [.16, .2, .01], [.65, 0, .19], [.27, .25, .22, 1]),
  fixedBox('hammer_shelf_support_north', [.025, .035, .037], [.65, .15, .143], [.17, .18, .19, 1]),
  fixedBox('hammer_shelf_support_south', [.025, .035, .037], [.65, -.15, .143], [.17, .18, .19, 1]),
  ...(includeTorqueDriverCradle ? [
    fixedBox('torque_driver_cradle_south', [.20, .008, .012], [.53, -.54, .130], [.17, .18, .19, 1]),
    fixedBox('torque_driver_cradle_north', [.20, .008, .012], [.53, -.30, .130], [.17, .18, .19, 1]),
  ] : []),
];

function createPatches(toolAssetXml, toolXml) {
  return [
    { target: 'panda.xml', replace: ['name="actuator8"', 'name="gripper"'] },
    {
      target: 'panda.xml',
      replace: [
        '<general class="panda" name="gripper" tendon="split" forcerange="-100 100" ctrlrange="0 255"\n      gainprm="0.01568627451 0 0" biasprm="0 -100 -10"/>',
        '<general class="panda" name="gripper" tendon="split" forcerange="-100 100" ctrlrange="0 255"\n      gainprm=".23529411765 0 0" biasprm="0 -1500 -40"/>',
      ],
    },
    {
      target: 'panda.xml',
      replace: [
        '<default class="fingertip_pad_collision_1">\n          <geom type="box" size="0.0085 0.004 0.0085" pos="0 0.0055 0.0445"/>\n        </default>',
        '<default class="fingertip_pad_collision_1">\n          <geom type="box" size="0.0085 0.004 0.0085" pos="0 0.0055 0.0445" friction="3 .2 .05" condim="6" solref=".002 1" solimp=".95 .99 .001"/>\n        </default>',
      ],
    },
    {
      target: 'panda.xml',
      inject: '<site name="tcp" pos="0 0 0.1" size="0.01" rgba="0.75 0.18 0.12 0.7" group="1"/>',
      injectAfter: '<body name="hand"',
    },
    {
      target: 'scene.xml',
      replace: [
        '  <include file="panda.xml"/>',
        `  <asset><model name="panda_model" file="panda.xml"/>${toolAssetXml}</asset>`,
      ],
    },
    { target: 'scene.xml', replace: ['  <worldbody>', `  <worldbody>${attachmentFrames()}`] },
    { target: 'scene.xml', replace: ['</worldbody>', `${SHARED_WORKCELL_XML}${toolXml}\n  </worldbody>`] },
    {
      target: 'panda.xml',
      replace: [
        '  <keyframe>\n    <key name="home" qpos="0 0 0 -1.57079 0 1.57079 -0.7853 0.04 0.04" ctrl="0 0 0 -1.57079 0 1.57079 -0.7853 255"/>\n  </keyframe>\n\n',
        '',
      ],
    },
  ];
}

function createLayout(toolAssetXml, toolXml, includeTorqueDriverCradle = false) {
  return {
    instanceCount: 4,
    yawStepDegrees: 90,
    ringRadius: RING_RADIUS,
    workSurfaceHeight: PLATFORM_TOP,
    primaryTcpSite: 'r0_tcp',
    primaryGripperActuator: 'r0_gripper',
    homeJoints: repeatPose(FRANKA_HOME, 4),
    taskStations: { ...TASK_STATIONS },
    xmlPatches: createPatches(toolAssetXml, toolXml),
    sceneObjects: sceneObjects(includeTorqueDriverCradle),
    camera: { position: [2.85, -2.85, 3.05], fov: 45 },
    orbitTarget: [0, 0, .32],
  };
}

export const FRANKA_ASSEMBLY1_LAYOUT = createLayout(
  ASSEMBLY1_ASSET_XML,
  ASSEMBLY1_TOOL_XML,
  true,
);
export const FRANKA_ASSEMBLY2_LAYOUT = createLayout(ASSEMBLY2_ASSET_XML, ASSEMBLY2_TOOL_XML);
