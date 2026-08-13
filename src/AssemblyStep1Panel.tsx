import type { AssemblyStep1Status } from './assemblyStep1.js';

const statusCopy: Record<AssemblyStep1Status, { button: string; status: string }> = {
  idle: { button: '执行第一步：协作就位', status: '就绪' },
  planning: { button: '正在规划四臂轨迹…', status: '规划中' },
  running: { button: '正在执行第一步…', status: '执行中' },
  complete: { button: '第一步已完成', status: '四臂已到达预抓取位' },
  error: { button: '第一步规划失败', status: '请 Reset 后重试' },
};

export function AssemblyStep1Panel({
  status,
  onRun,
}: {
  status: AssemblyStep1Status;
  onRun: () => void;
}) {
  const copy = statusCopy[status];
  return (
    <section className="assembly-step1-panel" aria-label="Assembly1 first action">
      <div className="assembly-step1-panel__title">Assembly1 动作</div>
      <div className="assembly-step1-panel__status">{copy.status}</div>
      <button
        type="button"
        onClick={onRun}
        disabled={status !== 'idle'}
      >
        {copy.button}
      </button>
    </section>
  );
}
