# Spare fastener stability

## Reproduction and cause

The idle browser reproduction identified `fastener_2` as the most visibly rotating spare. The regression test measured 110.75 degrees of orientation drift before the fix, without starting any robot action. Both spares rested on `fastener_tray_floor`; their cylindrical shafts used default `condim=3`, which does not resist spin around the contact normal. Initial shaft bottoms also started 7 mm above the tray.

Reference: [MuJoCo contact dimensionality](https://mujoco.readthedocs.io/en/stable/XMLreference.html#body-geom-condim): `condim=4` adds torsional friction; `condim=6` also adds rolling friction.

## Minimal change

- Only the Assembly1 spare shafts (`fastener_2_shaft`, `fastener_3_shaft`) gain `condim=6`, with friction coefficients `1 .005 .001`, priority 1, and a .004 s critically damped contact time constant. The existing Assembly1 workcell conversion makes final `solimp` equal to `.99 .999 .0001`. This stiffens only spare-shaft contacts, not the tray or picked pin's contact law.
- Spare body initial height changes from .152 m to .145 m, placing the .025 m half-length shaft on the .12 m tray surface.
- No mass, shape, free-joint, grasp-target, controller or trajectory changes. No freezing, pose writes, velocity resets, adhesion or disabled collisions.
- The picked `fastener_1` is unchanged. Assembly1 and Demo1 use the same corrected layout.

## Validation

- Regression script: `scripts/verify-spare-fastener-stability.mjs`; failed before the fix, passed afterward in idle simulation.
- First attempt (`condim=4` only) passed short idle observation, but failed the long playback stability check: spare 2 drifted 126.82 degrees / 3.09 mm; the task itself completed. This was not accepted as a fix. Rolling contact resistance and firmer contact settling were then added locally to the spare shafts.
- Final idle observation after initial settling: 400 samples over 17.01 s; maximum orientation drift 0.026 degrees / 0.049 degrees; translation 0.011 mm / 0.021 mm for spares 2 / 3. Both remain supported by actual tray contacts. These are finite numerical residuals, not a claim of exactly zero motion.
- TypeScript and production build passed; 211/211 unit tests passed.
- Full continuous playback completed all four steps. Over the 356.45 s observation window (1489 samples after warm-up), maximum orientation changes were 0.0348 degrees / 0.0569 degrees for spares 2 / 3. No sampled contacts with objects other than the tray; all MuJoCo warning counters stayed zero. The reported persistent axial spinning is suppressed.
- The stricter translational guard **did not pass**: maximum accumulated translation was 0.3758 mm / 0.1058 mm, exceeding the script's 0.1 mm threshold. This is residual slow numerical drift, distinct from the original large axial rotation. The guard was not relaxed, and the overall full-playback stability script must not be described as passing. Further elimination of that residual drift remains open.
- [Idle measurements](../../artifacts/reports/spare-fasteners-idle.json)
- [Full-playback measurements, including residual drift](../../artifacts/reports/spare-fasteners-demo.json)
- [Close-up recording](../../artifacts/videos/spare-fasteners-stable.webm)
- [Close-up screenshot](../../artifacts/screenshots/spare-fasteners-stable.png)

The user subsequently requested committing and pushing this change together with the palette restoration. The residual-drift limitation above remains open.
