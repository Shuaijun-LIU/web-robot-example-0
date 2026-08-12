# Skill Routing (suggestions only)

## Typical order (high level)
1) `project-flow-manager` (initialize + track)
2) `hf-model-localizer` (download/localize model checkpoints)
3) `paper-literature-packet` (related work packet + BibTeX)
4) `experiment-runpack` (run folders + seeds + env capture + reproducibility packaging)
5) `data-quality-eda-results` (data QC + results tables)
6) `paper-writer-6p` or `paper-writer-8to10p` (write in IEEE template)
7) `paper-vector-figures` / `project-schematics` (figures)
8) `github-repo-coordinator` (commit/PR/checks)

## Artifact handoff (recommended)
- Copy `lit_packet/06_references.bib` → `paper/references.bib`
- Put final figures under `paper/figures/`
- Keep numeric results in `project/data_packet.md` and/or `project/results/results_table.md`

## Project management
- Use: `project-flow-manager` (tracking + reminders)

## Model acquisition
- Use: `hf-model-localizer`

## Experiment execution / reproducibility packaging
- Use: `experiment-runpack`

## Data checks / EDA / results tables
- Use: `data-quality-eda-results`

## Literature / related work
- Use: `paper-literature-packet`

## Writing
- Use: `paper-writer-6p` (workshop/short)
- Use: `paper-writer-8to10p` (full/journal-style)

## Figures
- Use: `paper-vector-figures` (SVG/PDF, plots)
- Use: `project-schematics` (architecture/structure diagrams)

## GitHub coordination
- Use: `github-repo-coordinator`
