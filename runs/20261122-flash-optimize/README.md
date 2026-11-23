# 2026-11-22 — Flash optimisation: course table in centimetres

Why: after the front-edge ToF layer the Release build sat at 65 476 B / 65 536 B
(60 B free). The hand-tuned course table alone cost 426 B (71 points × 6 B).

What changed: the table is now stored at 2 B per point (`uint8` cm x, `int8` cm y)
and expanded to millimetres into the RAM copy by `CourseRoute_Init()` at startup.

| | before | after |
|---|---|---|
| FLASH | 65 476 B (99.91 %, 60 B free) | 65 288 B (99.62 %, 248 B free) |
| RAM | 17 192 B (83.95 %) | 17 616 B (86.02 %) |
| HEX bytes | 184 238 | 183 719 |
| HEX SHA-256 | `C2B063DC…8E26` | `C96F0A77…F2AF` |

Net flash saving is 188 B, not 284 B: the 284 B table saving is partly spent on
the 96 B expansion loop.

## Artifacts
- `apply_route_cm.py` — rewrites `Inc/course_route.h` by line markers.
  `python apply_route_cm.py` applies, `python apply_route_cm.py verify` expands
  the cm table and compares it against `route_mm_before.txt`.
- `route_mm_before.txt` — the original 71-point millimetre table (ground truth).
- `course_route_cm_block.txt` — generated C block (reference copy).
- `course_route.h.mm.bak` — the header as it was before the table replacement.
  Note: its comment block already described the cm layout, so the comment is
  stale relative to the millimetre table it contains.
- `apply.txt` — output of the apply + verify run.

## Numbers from verify
- 71 points, leg changes at indices 14 and 69 (legs 1, 2, 3).
- Worst rounding error: 5 mm on one axis.
- Finish point: (1280, −70) instead of (1275, −65) — 35 mm to the U block edge,
  75 mm to T15.

## Revert
1. Copy `course_route.h.mm.bak` over `code_test_xetranhvatcan/Inc/course_route.h`.
2. Delete the `CourseRoute_Init();` call at the top of `Navigation_Init()` in
   `Src/navigation.c` (and revert the arrival radius 12 → 8 mm if wanted).
3. Rebuild with `runs/build_release.ps1`.

## Verified
- `runs/build_release.ps1` — exit 0, numbers above.
- `Tests/run_checks.ps1` — 9/9 groups PASS.
- `runs/20261122-front-edge-avoid/validate_beam.py` — closed loop still settles
  at X = 1274.9 mm for drift 0/88/120 mm.
- Not flashed to the car yet.
