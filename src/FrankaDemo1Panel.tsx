import type { FrankaDemo1Action } from './frankaDemo1.js';
import { frankaDemo1StatusText } from './frankaDemo1.js';

interface FrankaDemo1PanelProps {
  action: FrankaDemo1Action | null;
  active: boolean;
  sceneReady: boolean;
  page: string;
  pages: Record<string, string>;
  onPageChange: (page: string) => void;
  onPlay: () => void;
  onReset: () => void;
}

export function FrankaDemo1Panel({
  action,
  active,
  sceneReady,
  page,
  pages,
  onPageChange,
  onPlay,
  onReset,
}: FrankaDemo1PanelProps) {
  const terminal = action === 'complete' || action === 'error';
  return (
    <section className="franka-demo1-panel" aria-label="Franka Demo1 controls">
      <div className="franka-demo1-panel__title">Franka Demo1</div>
      <label className="franka-demo1-panel__page">
        Page
        <select value={page} onChange={(event) => onPageChange(event.target.value)}>
          {Object.entries(pages).map(([label, value]) => (
            <option key={value} value={value}>{label}</option>
          ))}
        </select>
      </label>
      <div className="franka-demo1-panel__actions">
        <button type="button" onClick={onPlay} disabled={!sceneReady || active || terminal}>
          Play
        </button>
        <button type="button" onClick={onReset}>Reset</button>
      </div>
      <div className={`franka-demo1-panel__status${action === 'error' ? ' franka-demo1-panel__status--error' : ''}`} aria-live="polite">
        {frankaDemo1StatusText(action)}
      </div>
    </section>
  );
}
