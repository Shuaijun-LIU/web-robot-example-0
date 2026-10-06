/** A key event changes one actuator; selection/mounting never writes ctrl. */
export function toggleCooperativeGripper(data,actuator,enabled){
  if(!enabled||!Number.isInteger(actuator)||actuator<0||actuator>=data.ctrl.length)return false;
  data.ctrl[actuator]=data.ctrl[actuator]>127.5?0:255;
  return true;
}
