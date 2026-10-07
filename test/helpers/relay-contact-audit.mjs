const local=(d,b,p)=>[0,1,2].map(k=>p.reduce((s,v,j)=>s+(v-d.xpos[b*3+j])*d.xmat[b*9+j*3+k],0));
const world=(d,b,p)=>[0,1,2].map(j=>d.xpos[b*3+j]+p.reduce((s,v,k)=>s+v*d.xmat[b*9+j*3+k],0));

/** Independently track MATERIAL surface points from actual contact pairs.
 * Does not trust mj_geomDistance's exact-zero mesh-query failure mode.
 * No scene state/force/attachment is changed by this observer.
 */
export class RelayContactAudit {
  constructor(m,d,panel,fingers){
    this.panel=panel;this.fingers=new Set(fingers.flat());this.witness=new Map();
  }
  update(m,d){
    this.currentPoints=[];
    const vector=d.contact,latest=new Map();
    try{for(let i=0;i<d.ncon;i++){
      const c=vector.get(i);
      try{
        const a=m.geom_bodyid[c.geom1],b=m.geom_bodyid[c.geom2];
        if(c.dist>0||![a,b].includes(this.panel))continue;
        const finger=a===this.panel?b:a;if(!this.fingers.has(finger))continue;
        const p1=Array.from(c.pos).map((v,j)=>v-c.frame[j]*c.dist/2);
        const p2=Array.from(c.pos).map((v,j)=>v+c.frame[j]*c.dist/2);
        this.currentPoints.push({finger,point:local(d,this.panel,a===this.panel?p1:p2)});
        if(latest.has(finger)&&latest.get(finger).depth<-c.dist)continue;
        latest.set(finger,{time:d.time,depth:-c.dist,panel:local(d,this.panel,a===this.panel?p1:p2),finger:local(d,finger,a===finger?p1:p2)});
      }finally{c.delete?.();}
    }}finally{vector.delete?.();}
    for(const [finger,value] of latest)this.witness.set(finger,value);
  }
  gap(d,finger){
    const w=this.witness.get(finger);if(!w||d.time-w.time>.0060001||d.time<w.time)return Infinity;
    const p=world(d,this.panel,w.panel),q=world(d,finger,w.finger);
    return Math.hypot(...p.map((v,j)=>v-q[j]));
  }
}

export function meshBounds(m,d,g){
  const mesh=m.geom_dataid[g],start=m.mesh_vertadr[mesh]*3,end=start+m.mesh_vertnum[mesh]*3;
  const lo=[Infinity,Infinity,Infinity],hi=[-Infinity,-Infinity,-Infinity];
  for(let v=start;v<end;v+=3)for(let j=0;j<3;j++){
    let p=d.geom_xpos[g*3+j];for(let k=0;k<3;k++)p+=d.geom_xmat[g*9+j*3+k]*m.mesh_vert[v+k];
    lo[j]=Math.min(lo[j],p);hi[j]=Math.max(hi[j],p);
  }
  return [lo,hi];
}

/** A positive gap is a separating-plane certificate, hence a lower bound. */
export const boundsSeparation=(a,b)=>Math.max(0,...[0,1,2].flatMap(j=>[a[0][j]-b[1][j],b[0][j]-a[1][j]]));
