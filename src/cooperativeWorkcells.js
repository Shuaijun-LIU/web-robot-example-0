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
};
export function cooperativeSceneForKey(key){
  return Object.entries(COOPERATIVE_WORKCELLS).find(([,scene])=>scene.key===key)?.[0]??null;
}
