import type {CooperativePlan,CooperativeState} from './cooperativeMotion.js';
export class CooperativeMotionRuntime {
  constructor(plan:CooperativePlan,find:{body(name:string):number;joint(name:string):number});
  state:CooperativeState;time:number;history:unknown[];metrics:Record<string,number>;
  start(model:unknown,data:unknown):void;step(model:unknown,data:unknown):void;reset():void;
}
