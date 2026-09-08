# Asset source/license audit — 2026-09-08

Distribution source: [RoboDojo-Benchmark/RoboDojo dataset](https://huggingface.co/datasets/RoboDojo-Benchmark/RoboDojo/tree/a14409d7fae673c00499e01fd88b4457df6351b1).

At revision `a14409d7fae673c00499e01fd88b4457df6351b1`, the dataset card and public API declare `license: apache-2.0`. The local USDZ files match the distribution's `X-Linked-ETag` SHA-256 values exactly:

| Asset path under `Assets/Object/RoboDojo/` | Source SHA-256 |
| --- | --- |
| `Rigid/egg/00000/object.usdz` | `867733884c64ee06b16374c7228e328824a16538d780f2ed2f61b889a8fc872d` |
| `Articulation/egg_holder/00000/object.usdz` | `417f7b067e13f5371c84164daf2d597dd29b7d96f7af8c3952758695e400f830` |

The applicable distribution license is retained in `RoboDojo-assets-Apache-2.0.txt`. Attribution: RoboDojo-Benchmark / RoboDojo contributors. Modified files in this package are conversions and adaptations: USD mesh extraction to OBJ, egg centering/color/size variants, holder-bottom extraction and scaling, convex collision decomposition. No endorsement by the upstream authors is implied.

Do not conflate the asset dataset with the separate GitHub code repository: its README describes non-commercial research while its LICENSE file contains MIT text. `RoboDojo.txt` records that code-repository text for historical provenance only; it is not used as the licensing basis for these asset files. This audit relies on the separately published asset dataset's explicit Apache-2.0 declaration and matching file hashes.
