import { useCallback, useState } from 'react';
import {
  createAssembly1PoseSnapshot,
  createManualPoseDownload,
} from './manualPoseCapture.js';

interface Assembly1PoseCapturePanelProps {
  sceneReady: boolean;
  selectedControlTarget: string;
  manualMode: boolean;
  onManualModeChange: (enabled: boolean) => void;
}

interface SaveResponse {
  ok?: boolean;
  path?: string;
  error?: string;
}

export function Assembly1PoseCapturePanel({
  sceneReady,
  selectedControlTarget,
  manualMode,
  onManualModeChange,
}: Assembly1PoseCapturePanelProps) {
  const [label, setLabel] = useState('handover-pose-01');
  const [saving, setSaving] = useState(false);
  const [message, setMessage] = useState('');
  const [hasError, setHasError] = useState(false);

  const saveCurrentPose = useCallback(async () => {
    setSaving(true);
    setMessage('');
    setHasError(false);
    try {
      if (!window.robotDemo) throw new Error('仿真诊断接口尚未就绪');
      const snapshot = createAssembly1PoseSnapshot(window.robotDemo, {
        label,
        selectedControlTarget,
      });
      const artifact = createManualPoseDownload(snapshot);
      const objectUrl = URL.createObjectURL(new Blob([artifact.contents], {
        type: artifact.mimeType,
      }));
      const anchor = document.createElement('a');
      anchor.href = objectUrl;
      anchor.download = artifact.filename;
      document.body.append(anchor);
      anchor.click();
      anchor.remove();
      URL.revokeObjectURL(objectUrl);

      if (import.meta.env.DEV) {
        const response = await fetch('/__manual-pose-capture', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify(snapshot),
        });
        const result = await response.json() as SaveResponse;
        if (!response.ok || !result.ok || !result.path) {
          throw new Error(result.error ?? `保存请求失败（HTTP ${response.status}）`);
        }
        setMessage(`已下载：${artifact.filename}；服务器已保存：${result.path}`);
      } else {
        setMessage(`已下载：${artifact.filename}`);
      }
    } catch (error) {
      setHasError(true);
      setMessage(error instanceof Error ? error.message : String(error));
    } finally {
      setSaving(false);
    }
  }, [label, selectedControlTarget]);

  return (
    <section className="assembly-pose-panel" aria-label="Assembly1 manual pose capture">
      <div className="assembly-pose-panel__title">手动定姿与采样</div>
      <div className="assembly-pose-panel__hint">
        先切换右上角 Control target，再用 IK gizmo / 键盘摆臂；保存后服务器可直接读取 JSON。
      </div>
      <button
        type="button"
        className={manualMode ? 'assembly-pose-panel__mode assembly-pose-panel__mode--active' : 'assembly-pose-panel__mode'}
        disabled={!sceneReady}
        onClick={() => onManualModeChange(!manualMode)}
      >
        {manualMode ? '退出手动定姿' : '进入手动定姿'}
      </button>
      <label className="assembly-pose-panel__label">
        姿态名称
        <input
          value={label}
          maxLength={64}
          spellCheck={false}
          onChange={(event) => setLabel(event.target.value)}
        />
      </label>
      <button
        type="button"
        disabled={!sceneReady || saving}
        onClick={saveCurrentPose}
      >
        {saving ? '正在保存…' : '保存当前姿态'}
      </button>
      <div
        className={hasError ? 'assembly-pose-panel__message assembly-pose-panel__message--error' : 'assembly-pose-panel__message'}
        aria-live="polite"
      >
        {message || `当前控制：${selectedControlTarget}`}
      </div>
    </section>
  );
}
