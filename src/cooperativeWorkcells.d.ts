export type CooperativeScene='scan'|'pot'|'drawer'|'insertion';
export interface CooperativeWorkcell {
  key:string;sceneFile:string;title:string;description:string;roles:string[];note:string;
  homeJoints:number[];camera:{position:[number,number,number];fov:number};orbitTarget:[number,number,number];
}
export const COOPERATIVE_WORKCELLS:Record<CooperativeScene,CooperativeWorkcell>;
export function cooperativeSceneForKey(key:string):CooperativeScene|null;
