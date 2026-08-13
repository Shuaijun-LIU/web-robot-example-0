# Home Lab Chair and TV Fit Design

Date: 2026-08-13

## Goal

Correct two visible furniture defects in `SO101 Home Lab`: replace the office
chair's stick-like arm pieces with complete supported armrests, and place the TV
and both speakers cleanly on the TV console without intersecting the console or
one another.

## Design

The chair keeps its current seat, back, base, and room position. Each armrest is
rebuilt from two slim vertical supports and one horizontal padded rail. The
supports connect to the seat area and stop beneath the rail; the rail runs along
the sitting direction and remains clear of the backrest.

The TV console retains its room center but grows along its width so the screen
and speakers fit as one composition. Its two front doors and handles expand
with the cabinet. The TV is centered through the cabinet depth and raised until
the bottom of its stand exactly meets the cabinet top. Each speaker is placed
on the same top plane, outside the screen width with a visible gap, and fully
inside the cabinet footprint.

## Verification

- Generated Home Lab MJCF contains four chair-arm supports and two padded rails.
- TV stand bottom and both speaker bottoms equal the console-top elevation.
- TV stand and speaker footprints remain inside the console-top footprint.
- Speaker footprints have positive clearance from the TV panel.
- MuJoCo compilation, initial-contact inspection, the project test suite,
  TypeScript, and production build pass.
- The refreshed 1440 × 900 screenshot is inspected for the requested visual
  placement.
