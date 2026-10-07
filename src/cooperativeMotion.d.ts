import type {CooperativeScene} from './cooperativeWorkcells.js';
export interface CooperativeState {phase:'loading'|'ready'|'running'|'complete'|'error';label:string;stage:number;reason?:string;active?:boolean}
export interface CooperativeGate {type:string;object?:string;arm?:number;body?:string;container?:string;radius?:number;minZ?:number;maxZ?:number;minX?:number;maxX?:number;scanner?:string;origin?:number[];axis?:number[];target?:number[];normal?:number[];joint?:string;min?:number;max?:number;maxSpeed?:number}
export interface CooperativePhase {name:string;duration:number;settleTimeout?:number;paths:number[][][];grippers:number[][];allowedContacts:string[][];carry:{object:string;arm:number}[];gates:CooperativeGate[]}
export interface CooperativePlan {version:1;scene:CooperativeScene;initialJoints:number[];initialObjects:{name:string;position:number[]}[];initialFixtureJoints?:{joint:string;position:number;tolerance:number}[];phases:CooperativePhase[]}
export const PANDA_LIMITS:number[][];
export function validateCooperativePlan(p:unknown,scene:CooperativeScene):string|null;
export function sampleCooperativePhase(p:CooperativePhase,fraction:number):{joints:number[][];grippers:number[]};
export function checkCooperativeGate(g:CooperativeGate,o:Record<string,unknown>):string|null;
export function loadCooperativePlan(fetcher:typeof fetch,scene:CooperativeScene,url:string,signal?:AbortSignal):Promise<CooperativePlan>;
export function requestCooperativeInspectionSteps(api:{step(n:number):void}|null,ticks:number):boolean;
