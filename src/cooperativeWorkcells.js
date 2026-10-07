import {repeatPose} from './sceneLayouts.js';

const common={
  homeJoints:repeatPose([1.570796,-.785398,0,-2.356194,0,1.570796,.785398,255],4),
  camera:{position:[2.1,-2.1,2.6],fov:42},orbitTarget:[0,0,.24],
};
export const COOPERATIVE_WORKCELLS={
  scan:{...common,key:'frankaDemo3',sceneFile:'scan.xml',title:'Product inspection & packing',
    description:'Present two products, inspect their faces with a handheld scanner, then pack and dispatch the order.',
    roles:['Present products','Operate scanner','Pack inspected items','Manage order tray'],
    note:'Inspection uses a geometric pose gate; no barcode decoding is claimed.'},
  pot:{...common,key:'frankaDemo4',sceneFile:'pot.xml',title:'Cooperative pot loading',
    description:'Two arms support the pot while the other two load solid ingredients through a shared opening.',
    roles:['Hold first handle','Load first ingredient','Hold opposite handle','Load second ingredient'],
    note:'Solid ingredients only. The source pot retains its open physical cavity.'},
  drawer:{...common,key:'frankaDemo5',sceneFile:'drawer.xml',title:'Drawer access & order kitting',
    description:'Open two drawers, retrieve the requested products, pack one order and close both drawers.',
    roles:['Open & close south drawer','Retrieve & pack tea','Open & close north drawer','Retrieve & pack coffee'],
    note:'Passive drawers move through real handle contact. The shared packing zone is entered in turn.'},
  insertion:{...common,key:'frankaDemo6',sceneFile:'insertion.xml',title:'Supported insertion & dispatch',
    description:'Supply a frame, stabilize its socket, insert it and transfer the assembled stand to the output.',
    roles:['Stabilize the stand','Align & insert the frame','Supply the frame','Transfer assembled stand'],
    note:'Both parts remain free objects. Insertion and transport rely on real contact.'},
  relay:{...common,key:'frankaDemo7',sceneFile:'relay.xml',title:'Panel relay over an obstacle',
    camera:{position:[1.55,-1.85,1.9],fov:42},orbitTarget:[0,.06,.24],
    description:'Lift, hand over, clear the barrier and place.',
    roles:['Lift south edge','Receive & carry east','Lift north edge','Receive & carry west'],
    note:'Contact first, release second. No attachment.'},
};
export function cooperativeSceneForKey(key){
  return Object.entries(COOPERATIVE_WORKCELLS).find(([,scene])=>scene.key===key)?.[0]??null;
}
