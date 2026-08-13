import type { AssemblyStep1Status } from './assemblyStep1.js';
import type { AssemblyStep2Phase, AssemblyStep2State } from './assemblyStep2.js';

const step1Copy: Record<AssemblyStep1Status, { button: string; status: string }> = {
  idle: { button: '执行第一步：协作就位', status: '就绪' },
  planning: { button: '正在规划四臂轨迹…', status: '规划中' },
  running: { button: '正在执行第一步…', status: '执行中' },
  complete: { button: '第一步已完成', status: '四臂已到达预抓取位' },
  error: { button: '第一步规划失败', status: '请 Reset 后重试' },
};

const step2PhaseCopy: Record<AssemblyStep2Phase, string> = {
  idle: '等待第一步完成',
  planning: '正在验证物理夹持前置条件',
  approach: '四臂接近夹持高度',
  'slow-descent': '四臂缓慢下降至接触位',
  'contact-settle': '开爪保持，等待四臂接触位收敛',
  'frame-clamp': 'Arm 1 正在夹持框架',
  'frame-verification': '正在验证框架双侧接触',
  'cross-member-clamp': 'Arm 3 / Arm 4 正在同步夹持横梁',
  'cross-member-verification': '正在验证横梁四指接触',
  'torque-driver-clamp': 'Arm 2 正在夹持电动扭矩工具',
  'tool-verification': '正在验证工具双侧接触',
  'clamped-hold': '正在验证四处稳定保持',
  complete: '第二步已完成：四处物理夹持已建立',
  error: '第二步失败',
};

function step2ButtonCopy(phase: AssemblyStep2Phase) {
  if (phase === 'idle') return '执行第二步：下降并物理夹持';
  if (phase === 'complete') return '第二步已完成：四处物理夹持已建立';
  if (phase === 'error') return '第二步执行失败';
  return '正在执行第二步…';
}

export function AssemblySequencePanel({
  step1Status,
  step2State,
  canRunStep2,
  onRunStep1,
  onRunStep2,
}: {
  step1Status: AssemblyStep1Status;
  step2State: AssemblyStep2State;
  canRunStep2: boolean;
  onRunStep1: () => void;
  onRunStep2: () => void;
}) {
  const first = step1Copy[step1Status];
  const failure = step2State.failure;
  return (
    <section className="assembly-sequence-panel" aria-label="Assembly1 action sequence">
      <div className="assembly-sequence-panel__title">Assembly1 动作</div>
      <div className="assembly-sequence-panel__step">
        <div className="assembly-sequence-panel__status">{first.status}</div>
        <button type="button" onClick={onRunStep1} disabled={step1Status !== 'idle'}>
          {first.button}
        </button>
      </div>
      <div className="assembly-sequence-panel__step">
        <div
          className={`assembly-sequence-panel__status${
            step2State.phase === 'error' ? ' assembly-sequence-panel__status--error' : ''
          }`}
        >
          {step2PhaseCopy[step2State.phase]}
          {failure && (
            <span>
              {`：${failure.armKey ?? '系统'} / ${failure.code}`}
              {failure.detail ? ` / ${failure.detail}` : ''}
              {'。请 Reset 后重试'}
            </span>
          )}
        </div>
        <button type="button" onClick={onRunStep2} disabled={!canRunStep2}>
          {step2ButtonCopy(step2State.phase)}
        </button>
      </div>
    </section>
  );
}
