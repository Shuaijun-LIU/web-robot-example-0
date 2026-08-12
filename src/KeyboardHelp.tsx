const isMac = typeof navigator !== 'undefined' && /Mac|iPhone|iPad/.test(navigator.userAgent);
const MOD = isMac ? '⌘' : 'Ctrl';

const HELP: Record<string, string[]> = {
  franka: [
    'V — Toggle gripper',
    'Drag gizmo — Move arm (IK)',
    `${MOD}+click — Drag body`,
    'Double-click — Select body',
  ],
  so101: [
    'W/S — Arm forward/back',
    'Q/E — Arm up/down',
    'A/D — Shoulder rotate',
    'R/F — Wrist pitch',
    'Z/C — Wrist roll',
    'V — Toggle gripper',
    'Double-click — Select body',
  ],
  xlerobot: [
    'W/S — Drive forward/back',
    'A/D — Turn left/right',
    '7/Y — Left shoulder rotate',
    '8/U 9/I — Left arm IK',
    '0/O — Left pitch  -/P — Left roll',
    'H/N J/M K/, L/. ;// — Right arm',
    'V/B — Toggle grippers (L/R)',
    'R/T F/G — Head pan/tilt',
    'Double-click — Select body',
  ],
};

export function KeyboardHelp({ robotKey }: { robotKey: string }) {
  const lines = HELP[robotKey];
  if (!lines) return null;

  return (
    <div className="pointer-events-none fixed bottom-4 left-4 rounded-lg bg-black/60 px-4 py-3 font-mono text-xs text-slate-300 backdrop-blur-sm">
      <div className="mb-1 font-semibold text-slate-100">Keyboard</div>
      {lines.map((l) => (
        <div key={l}>{l}</div>
      ))}
    </div>
  );
}
