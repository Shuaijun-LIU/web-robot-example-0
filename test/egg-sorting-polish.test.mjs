import test from 'node:test';
import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import * as sorting from '../src/eggSorting.js';

// A boundary mismatch here changes which physical contacts are permitted.
test('contact-stage lookup agrees with actuator sampling at phase and lease boundaries',()=>{
  assert.equal(typeof sorting.sortingStageAt,'function','a stage-only lookup is missing');
  const plan=JSON.parse(readFileSync('public/assets/franka-egg-sorting/sorting-motion.json'));
  for(const task of plan.tasks){
    let time=task.start;
    assert.equal(sorting.sortingStageAt(task,time-.01),-1);
    for(const phase of task.phases){
      for(const t of [time-1e-10,time,time+1e-10,time+phase.duration/2,time+phase.duration]){
        assert.equal(sorting.sortingStageAt(task,t),sorting.sampleSortingTask(task,t)?.stage??-1);
      }
      time+=phase.duration;
    }
    assert.equal(sorting.sortingStageAt(task,time+1),10);
  }
});
