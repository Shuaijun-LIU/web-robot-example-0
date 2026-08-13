import { fixedBox, repeatPose } from './sceneLayouts.js';

const QUARTER_TURN_DEGREES = 90;
const HALF_TURN_DEGREES = 180;
const WORK_SURFACE_HEIGHT = 0.8;
const XLEROBOT_ARM_BASE_HEIGHT = 0.775;

const SO101_HOME = [0.0158, 2.052, 2.1307, -0.0845, 1.5857, -0.3745];
const XLEROBOT_HOME = [
  0,
  0,
  1.5708,
  1.5785,
  1.5777,
  0.0008,
  1.57,
  -0.25,
  -1.5708,
  1.5785,
  1.5777,
  0.0008,
  1.57,
  -0.25,
  0,
  0,
];

function attachmentFrames(model, body, poses) {
  return poses
    .map(({ position, yaw }, index) => {
      const euler = yaw === 0 ? '' : ` euler="0 0 ${yaw}"`;
      return `<frame pos="${position.join(' ')}"${euler}><attach model="${model}" body="${body}" prefix="r${index}_"/></frame>`;
    })
    .join('');
}

function radialGearGeometry(name, outerRadius, rgba) {
  const segmentCount = 12;
  const toothCount = 16;
  const innerRadius = 0.0055;
  const ringCenter = (innerRadius + outerRadius) / 2;
  const radialHalfSize = (outerRadius - innerRadius) / 2;
  const tangentialHalfSize = ringCenter * Math.sin(Math.PI / segmentCount) * 0.93;
  const segmentGeoms = Array.from({ length: segmentCount }, (_, index) => {
    const angle = (index * 2 * Math.PI) / segmentCount;
    const x = (ringCenter * Math.cos(angle)).toFixed(5);
    const y = (ringCenter * Math.sin(angle)).toFixed(5);
    const yaw = ((angle * 180) / Math.PI).toFixed(3);
    return `<geom name="${name}_ring_segment_${index}" type="box" pos="${x} ${y} 0" size="${radialHalfSize.toFixed(5)} ${tangentialHalfSize.toFixed(5)} 0.005" euler="0 0 ${yaw}" rgba="${rgba}" mass="0.003" friction="1.1 0.08 0.002"/>`;
  }).join('');
  const toothGeoms = Array.from({ length: toothCount }, (_, index) => {
    const angle = (index * 2 * Math.PI) / toothCount;
    const radius = outerRadius + 0.002;
    const x = (radius * Math.cos(angle)).toFixed(5);
    const y = (radius * Math.sin(angle)).toFixed(5);
    const yaw = ((angle * 180) / Math.PI).toFixed(3);
    return `<geom name="${name}_tooth_${index}" type="box" pos="${x} ${y} 0" size="0.003 0.0022 0.005" euler="0 0 ${yaw}" rgba="${rgba}" mass="0.001" friction="1.1 0.08 0.002"/>`;
  }).join('');
  return `${segmentGeoms}${toothGeoms}<site name="${name}_bore_site" pos="0 0 0" size="0.0045" rgba="0.52 0.16 0.12 0.45" group="1"/>`;
}

function perforatedPlateCollision(prefix, halfX, halfY, holeX, halfZ, rgba) {
  const holeHalf = 0.0075;
  const edgeSpan = halfX - (holeX + holeHalf);
  const centerSpan = holeX - holeHalf;
  const yStripHalf = (halfY - holeHalf) / 2;
  const yStripCenter = holeHalf + yStripHalf;
  return `
      <geom name="${prefix}_floor_left" type="box" pos="${-(holeX + holeHalf + edgeSpan / 2)} 0 0" size="${edgeSpan / 2} ${halfY} ${halfZ}" rgba="${rgba}" mass="0.025"/>
      <geom name="${prefix}_floor_center" type="box" pos="0 0 0" size="${centerSpan} ${halfY} ${halfZ}" rgba="${rgba}" mass="0.03"/>
      <geom name="${prefix}_floor_right" type="box" pos="${holeX + holeHalf + edgeSpan / 2} 0 0" size="${edgeSpan / 2} ${halfY} ${halfZ}" rgba="${rgba}" mass="0.025"/>
      <geom name="${prefix}_floor_input_north" type="box" pos="${-holeX} ${yStripCenter} 0" size="${holeHalf} ${yStripHalf} ${halfZ}" rgba="${rgba}" mass="0.012"/>
      <geom name="${prefix}_floor_input_south" type="box" pos="${-holeX} ${-yStripCenter} 0" size="${holeHalf} ${yStripHalf} ${halfZ}" rgba="${rgba}" mass="0.012"/>
      <geom name="${prefix}_floor_intermediate_north" type="box" pos="${holeX} ${yStripCenter} 0" size="${holeHalf} ${yStripHalf} ${halfZ}" rgba="${rgba}" mass="0.012"/>
      <geom name="${prefix}_floor_intermediate_south" type="box" pos="${holeX} ${-yStripCenter} 0" size="${holeHalf} ${yStripHalf} ${halfZ}" rgba="${rgba}" mass="0.012"/>`;
}

const SO101_PARENT_WORLDBODY = `  <worldbody>
    <!-- Manipulable objects for SO101 single arm -->
    <body name="sphere1" pos="0.55 -0.6 1">
      <freejoint/>
      <geom type="sphere" size="0.02" material="obj_green" mass="0.04"
            contype="1" conaffinity="1" friction="1.2 0.05 0.01" condim="4"/>
    </body>

    <body name="cylinder1" pos="0.45 -0.6 1">
      <freejoint/>
      <geom type="cylinder" size="0.02 0.06" material="obj_blue" mass="0.04"
            contype="1" conaffinity="1" friction="1.2 0.05 0.01" condim="4"/>
    </body>

    <body name="box2" pos="0.25 -0.6 1">
      <freejoint/>
      <geom type="box" size="0.015 0.015 0.03" material="obj_yellow" mass="0.04"
            contype="1" conaffinity="1" friction="1.2 0.05 0.01" condim="4"/>
    </body>

    <body name="sphere2" pos="0.15 -0.6 1">
      <freejoint/>
      <geom type="sphere" size="0.025" material="obj_red" mass="0.04"
            contype="1" conaffinity="1" friction="1.2 0.05 0.01" condim="4"/>
    </body>
  </worldbody>`;

const SO101_GEARBOX_WORKCELL_XML = `
    <body name="gearbox_fixture" pos="0 0 0.806">
      <geom name="gearbox_fixture_base" type="box" size="0.09 0.09 0.006" rgba="0.24 0.27 0.28 1"/>
      <geom name="gearbox_fixture_nw" type="box" pos="-0.071 0.056 0.018" size="0.009 0.012 0.018" rgba="0.12 0.14 0.15 1"/>
      <geom name="gearbox_fixture_ne" type="box" pos="0.071 0.056 0.018" size="0.009 0.012 0.018" rgba="0.12 0.14 0.15 1"/>
      <geom name="gearbox_fixture_sw" type="box" pos="-0.071 -0.056 0.018" size="0.009 0.012 0.018" rgba="0.12 0.14 0.15 1"/>
      <geom name="gearbox_fixture_se" type="box" pos="0.071 -0.056 0.018" size="0.009 0.012 0.018" rgba="0.12 0.14 0.15 1"/>
      <site name="fixture_center_site" pos="0 0 0.013" size="0.005" rgba="0.55 0.2 0.15 0.5" group="1"/>
    </body>

    <body name="gearbox_lower_housing" pos="0 0.21 0.805">
      <freejoint/>
      <geom name="housing_floor_visual" type="box" size="0.065 0.05 0.004" rgba="0.48 0.5 0.5 1" contype="0" conaffinity="0"/>
      ${perforatedPlateCollision('housing', 0.065, 0.05, 0.027, 0.004, '0.48 0.5 0.5 1')}
      <geom name="housing_wall_north" type="box" pos="0 0.047 0.019" size="0.065 0.003 0.015" rgba="0.37 0.39 0.4 1" mass="0.035"/>
      <geom name="housing_wall_south" type="box" pos="0 -0.047 0.019" size="0.065 0.003 0.015" rgba="0.37 0.39 0.4 1" mass="0.035"/>
      <geom name="housing_wall_west" type="box" pos="-0.062 0 0.019" size="0.003 0.044 0.015" rgba="0.37 0.39 0.4 1" mass="0.03"/>
      <geom name="housing_wall_east" type="box" pos="0.062 0 0.019" size="0.003 0.044 0.015" rgba="0.37 0.39 0.4 1" mass="0.03"/>
      <site name="housing_input_bearing_site" pos="-0.027 0 0.006" size="0.005" rgba="0.55 0.2 0.15 0.55" group="1"/>
      <site name="housing_intermediate_bearing_site" pos="0.027 0 0.006" size="0.005" rgba="0.55 0.2 0.15 0.55" group="1"/>
    </body>

    <body name="shaft_rack_input" pos="0.18 0 0.806">
      <geom name="shaft_rack_input_base" type="box" size="0.022 0.058 0.006" rgba="0.22 0.24 0.25 1"/>
      <geom name="shaft_rack_input_left" type="box" pos="-0.013 -0.032 0.013" size="0.006 0.012 0.006" euler="0 35 0" rgba="0.31 0.33 0.34 1"/>
      <geom name="shaft_rack_input_right" type="box" pos="0.013 -0.032 0.013" size="0.006 0.012 0.006" euler="0 -35 0" rgba="0.31 0.33 0.34 1"/>
      <geom name="shaft_rack_input_left_far" type="box" pos="-0.013 0.032 0.013" size="0.006 0.012 0.006" euler="0 35 0" rgba="0.31 0.33 0.34 1"/>
      <geom name="shaft_rack_input_right_far" type="box" pos="0.013 0.032 0.013" size="0.006 0.012 0.006" euler="0 -35 0" rgba="0.31 0.33 0.34 1"/>
    </body>
    <body name="shaft_rack_intermediate" pos="0.235 0 0.806">
      <geom name="shaft_rack_intermediate_base" type="box" size="0.022 0.053 0.006" rgba="0.22 0.24 0.25 1"/>
      <geom name="shaft_rack_intermediate_left" type="box" pos="-0.013 -0.028 0.013" size="0.006 0.012 0.006" euler="0 35 0" rgba="0.31 0.33 0.34 1"/>
      <geom name="shaft_rack_intermediate_right" type="box" pos="0.013 -0.028 0.013" size="0.006 0.012 0.006" euler="0 -35 0" rgba="0.31 0.33 0.34 1"/>
      <geom name="shaft_rack_intermediate_left_far" type="box" pos="-0.013 0.028 0.013" size="0.006 0.012 0.006" euler="0 35 0" rgba="0.31 0.33 0.34 1"/>
      <geom name="shaft_rack_intermediate_right_far" type="box" pos="0.013 0.028 0.013" size="0.006 0.012 0.006" euler="0 -35 0" rgba="0.31 0.33 0.34 1"/>
    </body>
    <body name="input_shaft" pos="0.18 0 0.824">
      <freejoint/>
      <geom name="input_shaft_body" type="cylinder" fromto="0 -0.045 0 0 0.045 0" size="0.004" rgba="0.58 0.59 0.6 1" mass="0.032" friction="1.25 0.08 0.002"/>
      <geom name="input_shaft_grip" type="cylinder" fromto="0 -0.016 0 0 0.016 0" size="0.006" rgba="0.23 0.25 0.26 1" mass="0.01"/>
    </body>
    <body name="intermediate_shaft" pos="0.235 0 0.824">
      <freejoint/>
      <geom name="intermediate_shaft_body" type="cylinder" fromto="0 -0.04 0 0 0.04 0" size="0.004" rgba="0.58 0.59 0.6 1" mass="0.029" friction="1.25 0.08 0.002"/>
      <geom name="intermediate_shaft_grip" type="cylinder" fromto="0 -0.014 0 0 0.014 0" size="0.006" rgba="0.23 0.25 0.26 1" mass="0.01"/>
    </body>
    <body name="spacer_input" pos="0.285 -0.025 0.812">
      <freejoint/>
      <geom name="spacer_input_body" type="cylinder" size="0.009 0.006" rgba="0.28 0.3 0.31 1" mass="0.018" friction="1.2 0.08 0.002"/>
    </body>
    <body name="spacer_intermediate" pos="0.285 0.025 0.812">
      <freejoint/>
      <geom name="spacer_intermediate_body" type="cylinder" size="0.009 0.006" rgba="0.28 0.3 0.31 1" mass="0.018" friction="1.2 0.08 0.002"/>
    </body>

    <body name="gear_parts_tray" pos="0 -0.21 0.804">
      <geom name="gear_parts_tray_floor" type="box" size="0.09 0.052 0.004" rgba="0.2 0.23 0.24 1"/>
      <geom name="gear_parts_tray_north" type="box" pos="0 0.05 0.009" size="0.09 0.003 0.009" rgba="0.16 0.18 0.19 1"/>
      <geom name="gear_parts_tray_south" type="box" pos="0 -0.05 0.009" size="0.09 0.003 0.009" rgba="0.16 0.18 0.19 1"/>
      <geom name="gear_parts_tray_west" type="box" pos="-0.088 0 0.009" size="0.003 0.047 0.009" rgba="0.16 0.18 0.19 1"/>
      <geom name="gear_parts_tray_east" type="box" pos="0.088 0 0.009" size="0.003 0.047 0.009" rgba="0.16 0.18 0.19 1"/>
      <geom name="gear_pocket_divider_a" type="box" pos="-0.03 0 0.006" size="0.002 0.047 0.006" rgba="0.16 0.18 0.19 1"/>
      <geom name="gear_pocket_divider_b" type="box" pos="0.03 0 0.006" size="0.002 0.047 0.006" rgba="0.16 0.18 0.19 1"/>
    </body>
    <body name="gear_large" pos="-0.057 -0.21 0.818">
      <freejoint/>
      ${radialGearGeometry('gear_large', 0.024, '0.55 0.43 0.18 1')}
    </body>
    <body name="gear_medium" pos="0 -0.21 0.818">
      <freejoint/>
      ${radialGearGeometry('gear_medium', 0.021, '0.48 0.49 0.5 1')}
    </body>
    <body name="gear_small" pos="0.05 -0.21 0.818">
      <freejoint/>
      ${radialGearGeometry('gear_small', 0.016, '0.36 0.38 0.39 1')}
    </body>

    <body name="gearbox_top_cover" pos="-0.21 0.025 0.809">
      <freejoint/>
      <geom name="cover_plate_visual" type="box" size="0.065 0.05 0.004" rgba="0.43 0.45 0.46 1" contype="0" conaffinity="0"/>
      ${perforatedPlateCollision('cover', 0.065, 0.05, 0.027, 0.004, '0.43 0.45 0.46 1')}
      <geom name="cover_grip_tab" type="box" pos="-0.077 0 0" size="0.012 0.028 0.008" rgba="0.24 0.26 0.27 1" mass="0.025"/>
      <site name="cover_input_bearing_site" pos="-0.027 0 0" size="0.005" rgba="0.55 0.2 0.15 0.55" group="1"/>
      <site name="cover_intermediate_bearing_site" pos="0.027 0 0" size="0.005" rgba="0.55 0.2 0.15 0.55" group="1"/>
    </body>
    <body name="pin_holder" pos="-0.21 -0.065 0.805">
      <geom name="pin_holder_base" type="box" size="0.082 0.018 0.005" rgba="0.19 0.21 0.22 1"/>
      <geom name="pin_holder_back" type="box" pos="0 0.016 0.012" size="0.082 0.003 0.012" rgba="0.15 0.17 0.18 1"/>
      <geom name="pin_holder_front" type="box" pos="0 -0.016 0.012" size="0.082 0.003 0.012" rgba="0.15 0.17 0.18 1"/>
      ${[-0.045, -0.015, 0.015, 0.045].map((x, index) => `
      <geom name="pin_holder_support_${index + 1}_left" type="box" pos="${x - 0.006} 0 0.012" size="0.002 0.012 0.012" rgba="0.27 0.29 0.3 1"/>
      <geom name="pin_holder_support_${index + 1}_right" type="box" pos="${x + 0.006} 0 0.012" size="0.002 0.012 0.012" rgba="0.27 0.29 0.3 1"/>`).join('')}
    </body>
    ${[-0.255, -0.225, -0.195, -0.165].map((x, index) => `
    <body name="press_pin_${index + 1}" pos="${x} -0.065 0.826">
      <freejoint/>
      <geom name="press_pin_${index + 1}_shaft" type="cylinder" size="0.003 0.016" rgba="0.57 0.58 0.59 1" mass="0.011" friction="1.2 0.08 0.002"/>
      <geom name="press_pin_${index + 1}_grip" type="cylinder" pos="0 0 0.012" size="0.0042 0.004" rgba="0.25 0.27 0.28 1" mass="0.003"/>
    </body>`).join('')}`;

export const SO101_GEARBOX_LAYOUT = {
  instanceCount: 4,
  yawStepDegrees: 90,
  ringRadius: 0.34,
  workSurfaceHeight: WORK_SURFACE_HEIGHT,
  primaryTcpSite: 'r0_tcp',
  homeJoints: repeatPose(SO101_HOME, 4),
  taskStations: {
    fixture: [0, 0, 0.81],
    housing: [0, 0.21, 0.805],
    shaftsAndSpacers: [0.21, 0, 0.82],
    gears: [0, -0.21, 0.814],
    coverAndPins: [-0.21, 0, 0.809],
  },
  xmlPatches: [
    {
      target: 'objects_SO101.xml',
      inject: '<model name="so101_model" file="SO101.xml"/>',
      injectAfter: '<asset>',
    },
    {
      target: 'objects_SO101.xml',
      replace: [
        SO101_PARENT_WORLDBODY,
        `  <worldbody>${attachmentFrames('so101_model', 'Base', [
          { position: [0, 0.34, 0.8], yaw: 0 },
          { position: [0.34, 0, 0.8], yaw: -QUARTER_TURN_DEGREES },
          { position: [0, -0.34, 0.8], yaw: HALF_TURN_DEGREES },
          { position: [-0.34, 0, 0.8], yaw: QUARTER_TURN_DEGREES },
        ])}${SO101_GEARBOX_WORKCELL_XML}\n  </worldbody>`,
      ],
    },
    {
      target: 'SO101.xml',
      inject: '<site name="tcp" pos="0 -0.04 -0.01" size="0.005" rgba="0.16 0.55 0.26 0.7" group="1"/>',
      injectAfter: '<body name="Fixed_Jaw"',
    },
    {
      target: 'SO101.xml',
      replace: [
        '<body name="Base" pos="0.35 -0.3 0.8" euler="0 0 0">',
        '<body name="Base" pos="0 0 0" euler="0 0 0">',
      ],
    },
    {
      target: 'SO101.xml',
      replace: [
        '  <keyframe>\n    <key name="home" qpos="0 0 0 0 0 -0.37453" ctrl="0 0 0 0 0 -0.37453"/>\n  </keyframe>\n',
        '',
      ],
    },
  ],
  sceneObjects: [
    fixedBox('gearbox_floor', [2, 2, 0.005], [0, 0, -0.005], [0.1, 0.11, 0.12, 1]),
    fixedBox('gearbox_work_surface', [0.52, 0.52, 0.035], [0, 0, 0.765], [0.31, 0.29, 0.25, 1]),
    fixedBox('gearbox_table_leg_a', [0.035, 0.035, 0.365], [-0.44, -0.44, 0.365], [0.18, 0.18, 0.17, 1]),
    fixedBox('gearbox_table_leg_b', [0.035, 0.035, 0.365], [0.44, -0.44, 0.365], [0.18, 0.18, 0.17, 1]),
    fixedBox('gearbox_table_leg_c', [0.035, 0.035, 0.365], [-0.44, 0.44, 0.365], [0.18, 0.18, 0.17, 1]),
    fixedBox('gearbox_table_leg_d', [0.035, 0.035, 0.365], [0.44, 0.44, 0.365], [0.18, 0.18, 0.17, 1]),
  ],
  camera: { position: [1.15, -1.15, 1.28], fov: 40 },
  orbitTarget: [0, 0, 0.81],
};

const XLEROBOT_PARENT_WORLDBODY = `  <worldbody>
    <!-- Manipulable objects with freejoint for physics simulation -->
    <body name="box1" pos="0.1 -0.4 1">
      <freejoint/>
      <geom type="box" size="0.015 0.02 0.05" material="obj_red" mass="0.1"
            contype="1" conaffinity="1" friction="1.2 0.05 0.01" condim="4" group="3"/>
    </body>

    <body name="sphere1" pos="0.5 -0.4 1">
      <freejoint/>
      <geom type="sphere" size="0.02" material="obj_green" mass="0.04"
            contype="1" conaffinity="1" friction="1.2 0.05 0.01" condim="4"/>
    </body>

    <body name="cylinder1" pos="0.4 -0.4 1">
      <freejoint/>
      <geom type="cylinder" size="0.02 0.06" material="obj_blue" mass="0.04"
            contype="1" conaffinity="1" friction="1.2 0.05 0.01" condim="4"/>
    </body>

    <body name="box2" pos="0.3 -0.4 1">
      <freejoint/>
      <geom type="box" size="0.04 0.04 0.04" material="obj_yellow" mass="0.04"
            contype="1" conaffinity="1" friction="1.2 0.05 0.01" condim="4"/>
    </body>

    <body name="sphere2" pos="0.2 -0.4 1">
      <freejoint/>
      <geom type="sphere" size="0.025" material="obj_red" mass="0.04"
            contype="1" conaffinity="1" friction="1.2 0.05 0.01" condim="4"/>
    </body>
  </worldbody>`;

const XLEROBOT_PAYLOAD_DECK_XML = `
      <body name="payload_deck" pos="-0.255 0 0.405">
        <geom name="payload_deck_floor" type="box" size="0.11 0.17 0.008" rgba="0.2 0.23 0.25 1" mass="0.45" friction="1.3 0.08 0.002"/>
        <geom name="payload_deck_left_rail" type="box" pos="0 0.166 0.028" size="0.11 0.004 0.028" rgba="0.17 0.19 0.2 1" mass="0.08"/>
        <geom name="payload_deck_right_rail" type="box" pos="0 -0.166 0.028" size="0.11 0.004 0.028" rgba="0.17 0.19 0.2 1" mass="0.08"/>
        <geom name="payload_deck_back_rail" type="box" pos="0.106 0 0.028" size="0.004 0.162 0.028" rgba="0.17 0.19 0.2 1" mass="0.06"/>
      </body>`;

const XLEROBOT_KITTING_WORKCELL_XML = `
    <body name="source_tote_bay" pos="-0.18 -0.3 0.777">
      <geom name="source_tote_bay_marker" type="box" size="0.14 0.11 0.002" rgba="0.33 0.3 0.24 1" contype="0" conaffinity="0"/>
    </body>
    <body name="order_tray_bay" pos="0.1 0.32 0.777">
      <geom name="order_tray_bay_marker" type="box" size="0.15 0.11 0.002" rgba="0.28 0.31 0.32 1" contype="0" conaffinity="0"/>
    </body>

    <body name="handoff_cradle_south" pos="0 -0.09 0.781">
      <geom name="handoff_cradle_south_base" type="box" size="0.06 0.035 0.006" rgba="0.2 0.23 0.24 1"/>
      <geom name="handoff_cradle_south_left" type="box" pos="-0.033 0 0.018" size="0.025 0.035 0.005" euler="0 -28 0" rgba="0.31 0.34 0.35 1"/>
      <geom name="handoff_cradle_south_right" type="box" pos="0.033 0 0.018" size="0.025 0.035 0.005" euler="0 28 0" rgba="0.31 0.34 0.35 1"/>
      <site name="handoff_south_site" pos="0 0 0.025" size="0.005" rgba="0.58 0.24 0.16 0.5" group="1"/>
    </body>
    <body name="handoff_cradle_north" pos="0 0.09 0.781">
      <geom name="handoff_cradle_north_base" type="box" size="0.06 0.035 0.006" rgba="0.2 0.23 0.24 1"/>
      <geom name="handoff_cradle_north_left" type="box" pos="-0.033 0 0.018" size="0.025 0.035 0.005" euler="0 -28 0" rgba="0.31 0.34 0.35 1"/>
      <geom name="handoff_cradle_north_right" type="box" pos="0.033 0 0.018" size="0.025 0.035 0.005" euler="0 28 0" rgba="0.31 0.34 0.35 1"/>
      <site name="handoff_north_site" pos="0 0 0.025" size="0.005" rgba="0.58 0.24 0.16 0.5" group="1"/>
    </body>
    <body name="scanner_dock" pos="0.22 0 0.781">
      <geom name="scanner_dock_base" type="box" size="0.075 0.055 0.006" rgba="0.18 0.2 0.21 1"/>
      <geom name="scanner_dock_north" type="box" pos="0 0.049 0.016" size="0.075 0.006 0.016" rgba="0.28 0.3 0.31 1"/>
      <geom name="scanner_dock_south" type="box" pos="0 -0.049 0.016" size="0.075 0.006 0.016" rgba="0.28 0.3 0.31 1"/>
      <site name="scanner_dock_site" pos="0 0 0.02" size="0.005" rgba="0.55 0.2 0.15 0.5" group="1"/>
    </body>

    <body name="source_tote" pos="-0.18 -0.3 0.781">
      <freejoint/>
      <geom name="source_tote_floor" type="box" size="0.13 0.1 0.006" rgba="0.26 0.29 0.3 1" mass="0.3" friction="1.25 0.08 0.002"/>
      <geom name="source_tote_north_wall" type="box" pos="0 0.096 0.045" size="0.13 0.004 0.04" rgba="0.22 0.25 0.26 1" mass="0.06"/>
      <geom name="source_tote_south_wall" type="box" pos="0 -0.096 0.045" size="0.13 0.004 0.04" rgba="0.22 0.25 0.26 1" mass="0.06"/>
      <geom name="source_tote_west_wall" type="box" pos="-0.126 0 0.045" size="0.004 0.092 0.04" rgba="0.22 0.25 0.26 1" mass="0.05"/>
      <geom name="source_tote_east_wall" type="box" pos="0.126 0 0.045" size="0.004 0.092 0.04" rgba="0.22 0.25 0.26 1" mass="0.05"/>
      <geom name="source_tote_handle_west" type="box" pos="-0.143 0 0.055" size="0.018 0.045 0.008" rgba="0.15 0.17 0.18 1" mass="0.025"/>
      <geom name="source_tote_handle_east" type="box" pos="0.143 0 0.055" size="0.018 0.045 0.008" rgba="0.15 0.17 0.18 1" mass="0.025"/>
    </body>

    <body name="pill_bottle" pos="-0.235 -0.34 0.821">
      <freejoint/>
      <geom name="pill_bottle_body" type="cylinder" size="0.016 0.029" rgba="0.83 0.82 0.76 1" mass="0.045" friction="1.15 0.06 0.002"/>
      <geom name="pill_bottle_cap" type="cylinder" pos="0 0 0.034" size="0.018 0.006" rgba="0.38 0.26 0.17 1" mass="0.012"/>
      <site name="pill_bottle_barcode_site" pos="0 -0.016 0" size="0.006 0.001 0.012" type="box" rgba="0.12 0.12 0.11 0.85" group="1"/>
    </body>
    <body name="tea_box" pos="-0.13 -0.34 0.827">
      <freejoint/>
      <geom name="tea_box_body" type="box" size="0.027 0.018 0.04" rgba="0.48 0.31 0.2 1" mass="0.065" friction="1.2 0.08 0.002"/>
      <geom name="tea_box_label" type="box" pos="0 -0.0185 0" size="0.018 0.001 0.026" rgba="0.72 0.65 0.49 1" contype="0" conaffinity="0"/>
      <site name="tea_box_barcode_site" pos="0.0275 0 0" size="0.001 0.012 0.018" type="box" rgba="0.12 0.12 0.11 0.85" group="1"/>
    </body>
    <body name="drink_carton" pos="-0.235 -0.255 0.833">
      <freejoint/>
      <geom name="drink_carton_body" type="box" size="0.023 0.018 0.045" rgba="0.63 0.5 0.34 1" mass="0.075" friction="1.2 0.08 0.002"/>
      <geom name="drink_carton_top" type="box" pos="0 0 0.05" size="0.021 0.016 0.006" rgba="0.75 0.69 0.57 1" mass="0.008"/>
      <geom name="drink_carton_cap" type="cylinder" pos="0.01 0 0.059" size="0.005 0.003" rgba="0.2 0.26 0.28 1" mass="0.003"/>
      <site name="drink_carton_barcode_site" pos="0 -0.0185 0" size="0.012 0.001 0.02" type="box" rgba="0.12 0.12 0.11 0.85" group="1"/>
    </body>
    <body name="water_bottle" pos="-0.13 -0.255 0.827">
      <freejoint/>
      <geom name="water_bottle_body" type="cylinder" size="0.017 0.034" rgba="0.42 0.52 0.55 0.9" mass="0.055" friction="1.15 0.06 0.002"/>
      <geom name="water_bottle_shoulder" type="capsule" fromto="0 0 0.028 0 0 0.047" size="0.012" rgba="0.42 0.52 0.55 0.9" mass="0.012"/>
      <geom name="water_bottle_cap" type="cylinder" pos="0 0 0.053" size="0.01 0.006" rgba="0.24 0.3 0.32 1" mass="0.008"/>
      <site name="water_bottle_barcode_site" pos="0 -0.017 0" size="0.007 0.001 0.014" type="box" rgba="0.12 0.12 0.11 0.85" group="1"/>
    </body>

    <body name="handheld_scanner" pos="0.22 0 0.812" euler="0 90 0">
      <freejoint/>
      <geom name="scanner_handle" type="box" pos="-0.032 0 -0.012" size="0.042 0.018 0.018" rgba="0.18 0.19 0.2 1" mass="0.09" friction="1.25 0.08 0.002"/>
      <geom name="scanner_head" type="box" pos="0.035 0 0.012" size="0.035 0.03 0.027" rgba="0.34 0.35 0.35 1" mass="0.11"/>
      <geom name="scanner_trigger" type="box" pos="-0.006 -0.019 -0.007" size="0.012 0.003 0.009" rgba="0.56 0.22 0.16 1" mass="0.004"/>
      <geom name="scanner_lens" type="box" pos="0.071 0 0.012" size="0.002 0.022 0.018" rgba="0.08 0.09 0.09 1" contype="0" conaffinity="0"/>
      <site name="scanner_lens_site" pos="0.074 0 0.012" size="0.004" rgba="0.55 0.16 0.12 0.65" group="1"/>
    </body>

    <body name="order_tray" pos="0.1 0.32 0.781">
      <freejoint/>
      <geom name="order_tray_floor" type="box" size="0.14 0.1 0.006" rgba="0.32 0.35 0.36 1" mass="0.32" friction="1.25 0.08 0.002"/>
      <geom name="order_tray_north_wall" type="box" pos="0 0.096 0.035" size="0.14 0.004 0.03" rgba="0.24 0.27 0.28 1" mass="0.055"/>
      <geom name="order_tray_south_wall" type="box" pos="0 -0.096 0.035" size="0.14 0.004 0.03" rgba="0.24 0.27 0.28 1" mass="0.055"/>
      <geom name="order_tray_west_wall" type="box" pos="-0.136 0 0.035" size="0.004 0.092 0.03" rgba="0.24 0.27 0.28 1" mass="0.045"/>
      <geom name="order_tray_east_wall" type="box" pos="0.136 0 0.035" size="0.004 0.092 0.03" rgba="0.24 0.27 0.28 1" mass="0.045"/>
      <geom name="order_tray_divider_a" type="box" pos="-0.045 0 0.025" size="0.003 0.092 0.02" rgba="0.23 0.26 0.27 1" mass="0.025"/>
      <geom name="order_tray_divider_b" type="box" pos="0.045 0 0.025" size="0.003 0.092 0.02" rgba="0.23 0.26 0.27 1" mass="0.025"/>
      <geom name="order_tray_handle_left" type="box" pos="-0.158 0 0.025" size="0.018 0.052 0.009" rgba="0.15 0.17 0.18 1" mass="0.025"/>
      <geom name="order_tray_handle_right" type="box" pos="0.158 0 0.025" size="0.018 0.052 0.009" rgba="0.15 0.17 0.18 1" mass="0.025"/>
      <site name="order_tray_center_site" pos="0 0 0.012" size="0.005" rgba="0.55 0.2 0.15 0.5" group="1"/>
    </body>`;

const xlerobotKittingTableObjects = [
  fixedBox('kitting_table_top', [0.36, 0.5, 0.025], [0, 0, 0.75], [0.31, 0.29, 0.25, 1]),
  fixedBox('kitting_table_leg_a', [0.035, 0.035, 0.3625], [-0.29, -0.43, 0.3625], [0.18, 0.18, 0.17, 1]),
  fixedBox('kitting_table_leg_b', [0.035, 0.035, 0.3625], [0.29, -0.43, 0.3625], [0.18, 0.18, 0.17, 1]),
  fixedBox('kitting_table_leg_c', [0.035, 0.035, 0.3625], [-0.29, 0.43, 0.3625], [0.18, 0.18, 0.17, 1]),
  fixedBox('kitting_table_leg_d', [0.035, 0.035, 0.3625], [0.29, 0.43, 0.3625], [0.18, 0.18, 0.17, 1]),
];

export const XLEROBOT_KITTING_LAYOUT = {
  instanceCount: 2,
  yawStepDegrees: 180,
  spacing: 2.2,
  armBaseHeight: XLEROBOT_ARM_BASE_HEIGHT,
  tableTopHeight: XLEROBOT_ARM_BASE_HEIGHT,
  chassisCollisionTop: XLEROBOT_ARM_BASE_HEIGHT,
  homeJoints: repeatPose(XLEROBOT_HOME, 2),
  taskStations: {
    sourceTote: [-0.18, -0.3, 0.781],
    handoffSouth: [0, -0.09, 0.787],
    handoffNorth: [0, 0.09, 0.787],
    scannerDock: [0.22, 0, 0.787],
    orderTray: [0.1, 0.32, 0.781],
  },
  xmlPatches: [
    {
      target: 'objects.xml',
      inject: '<model name="xlerobot_model" file="xlerobot.xml"/>',
      injectAfter: '<asset>',
    },
    {
      target: 'objects.xml',
      replace: [
        XLEROBOT_PARENT_WORLDBODY,
        `  <worldbody>${attachmentFrames('xlerobot_model', 'chassis', [
          { position: [-1.1, 0, 0], yaw: HALF_TURN_DEGREES },
          { position: [1.1, 0, 0], yaw: 0 },
        ])}${XLEROBOT_KITTING_WORKCELL_XML}\n  </worldbody>`,
      ],
    },
    {
      target: 'xlerobot.xml',
      replace: [
        '<geom type="box" size="0.1575 0.2025 0.295" pos="0 0 0.015" class="collision" friction="1 0.005 0.0001" rgba="0.9 0.3 0.3 0.4"/>',
        '<geom name="chassis_rack_collision" type="box" size="0.21 0.24 0.3375" pos="0 0 0.0575" class="collision" friction="1 0.005 0.0001" rgba="0 0 0 0"/>',
      ],
    },
    {
      target: 'xlerobot.xml',
      inject: XLEROBOT_PAYLOAD_DECK_XML,
      injectAfter: '<freejoint/',
    },
  ],
  tableObjects: xlerobotKittingTableObjects,
  sceneObjects: [
    fixedBox('kitting_floor', [3, 3, 0.005], [0, 0, -0.005], [0.1, 0.11, 0.12, 1]),
    ...xlerobotKittingTableObjects,
  ],
  camera: { position: [2.45, -2.75, 1.85], fov: 43 },
  orbitTarget: [0, 0, 0.68],
};
