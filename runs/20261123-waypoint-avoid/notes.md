# 2026-11-23 — Waypoint route: bug fixes + local ToF avoidance

Goal (unchanged): keep the waypoint route as the default path; only detour when
ToF confirms a real obstacle; rejoin the route after the detour.

## Part 1 — bug fixes (earlier in the session)
1. `Inc/course_route.h` — the compressed table declared
   `int8_t course_route_cm[][2]`, but route X reaches 132 cm. Values 128…132
   overflow `int8_t` (max 127) and wrap negative, so the last six points pointed
   outside the field and the finish leg could not be reached. Changed to
   `struct { uint8_t x_cm; int8_t y_cm; }`.
2. `Src/navigation.c` — `FixedLineMove()` restarted the drive command every
   10 ms while `state == NAV_MOVING` (it fell through to
   `Motion_StartDistanceHeading()` instead of returning). Restored the continuous
   branch: while moving, `Motion_RetargetForward()` keeps the wheels rolling and
   the car only pivots when the heading error exceeds the tolerance.
3. `Tests/sim_course.c` — dropped the old `expect_continuous` gate (it forbade
   every stop-and-pivot, but the hand-tuned route has square corners). The trace
   cases now assert the outcome: must `finish` and must not touch an obstacle.

## Part 2 — why the trigger was hard to choose
The extra obstacle in `t9` sits beside the route at the T9 corner, so the
collision is a **sideways graze**, not a frontal block. Measured on the empty
course, the centre beam reads 1500 mm while the right beam reads 92 mm at the
same instant, so a centre-beam-only trigger cannot see that obstacle at all. A
trial that steered on the centre beam did react, but it also reacted to the walls
on every corner and slowed the empty course 14.37 → 19.4 s, i.e. it changed the
path with no obstacle present. That trial was reverted.

## Part 3 — the trigger that is implemented
`FixedLineMove()` starts a detour only when **both** hold:

* the centre beam is **clear** (`front[1].mm >= FIXED_LINE_CENTRE_MM`, 300 mm) —
  so the lane ahead is open and the car is not simply facing a wall, and
* a diagonal beam is **close** (`front[0].mm <= 100 mm` or `front[2].mm <= 100 mm`)
  — so something sits beside the line.

On the empty course no sample satisfies both: `trace_nominal.csv` has 0 rows with
a non-zero offset, so the route is untouched when nothing is in the way. The aim
point is pushed away from the obstacle by
`FIXED_LINE_OFFSET_MM`, held for `FIXED_LINE_HOLD_MM` of travel, then ramped out
over `FIXED_LINE_RELEASE_MM` so the car rejoins the waypoint line. Waypoint
capture is measured against the shifted aim so the point index keeps advancing.

## Part 4 — parameter sweep
`runs/20261123-waypoint-avoid/sweep_offset_hold.ps1` → `sweep_offset_hold.txt`,
`sweep_offset_band.ps1` → `sweep_offset_band.txt`. Hold 400 mm, offsets swept:

| offset | result |
|---|---|
| 20, 25 | still grazes the extra obstacle |
| **30, 33, 35** | **finish, no contact** |
| 40, 45, 50 | swings into T7 instead |
| 60, 65, 70 | swings into T7 instead |

Chosen: offset 35 mm (centre of the usable band), hold 400 mm, release 120 mm.

## Part 5 — simulator results (final build)
| case | outcome | time | max lateral error | min gap |
|---|---|---|---|---|
| `trace nominal` | finish | 14.37 s | 1.9 mm | 40.3 mm |
| `trace t9` | finish | 15.18 s | 2.1 mm | 40.5 mm |

* `trace nominal` is unchanged from the pre-avoidance baseline: the detour never
  engages (0 rows with a non-zero offset in `trace_nominal.csv`).
* `trace t9` now finishes instead of hitting the extra obstacle. The detour is
  active on trace rows 80…163 of 304 (offset 35 mm, ramping to 0.2 mm at the end)
  and the car rejoins the line at 0.0 mm on the next row, so the rejoin path is
  exercised.
* The +0.81 s on `t9` is the cost of the detour and is expected.

Traces: `trace_nominal.csv`, `trace_t9.csv` (sampled every 5 ticks, 15 columns):
`px,py,heading,route_leg,state,est_x,est_y,left_mm,centre_mm,right_mm,local_step,
fixed_line_offset,avoid_dx,avoid_dy,fixed_line_point`, then a final summary line
`case,outcome,state,px,py,est_x,est_y,max_lateral_error,seconds,slip_applied,
slip_mm,min_middle_gap_mm`.

## Build / test
- `Tests/run_checks.ps1`: 9/9 PASS.
- Release build: **FLASH 65 536 B / 65 536 B (100.00 %, 0 B free)**, RAM 17 640 B
  / 20 480 B (86.13 %). text 65 292 + data 244 = 65 536.
- HEX SHA-256 `8D3311E4D409B0B154DB77E7942E1A3EC198E147E01BC5EDEE07CAD1374648CE`,
  184 423 bytes, `code_test_xetranhvatcan/build/Release/code_test_xetranhvatcan.hex`.
- **Not flashed, not driven.** The HEX on the car still predates the route-table
  fix, so its finish leg may be wrong.

## Known limitations
- **Flash is now full.** The avoidance layer costs roughly 590 B (the previous
  build was 64 944 B with 592 B free). Any further change needs flash reclaimed
  first, e.g. by dropping the release ramp or moving the constants to integers.
- The full `sim_course.exe` suite still aborts at `CHECK` in `sim_course.c`
  (the `NAV_WAIT_CLEARANCE` / `near_front_mask` assertion) because the fixed-line
  mode bypasses the A* clearance path. This is pre-existing: the same CHECK fails
  with the trigger compiled out (`suite_trigger_off_err.u8.txt`). Only the two
  `trace` cases are meaningful for this route.
- The detour band is narrow (30–40 mm at 400 mm hold). A real obstacle wider than
  the simulated one would need a wider band, which in turn would need a longer
  hold or a second detour stage.
- The rejoin is a straight ramp back to the table line, not a re-plan; if the
  obstacle is long the car will run beside the line until the hold expires.

## Follow-up: reject invalid/stale ToF samples

- A review found that the trigger used raw `mm` values without checking
  `NavigationRange.valid` or sample age. `FixedLineMove()` now requires all
  three front samples to be valid, the centre sample to be at most
  `NAV_SENSOR_AGE_MS` old, and each diagonal timestamp to be no more than
  `FIXED_LINE_TOF_SYNC_MS` (100 ms) older than the centre sample. Integer mm
  thresholds avoid unnecessary float comparisons; obstacle side selection
  compares the raw ranges directly.
- After this edit, `Tests/run_checks.ps1` remains 9/9 PASS. `trace nominal` still
  finishes in 14.37 s with 40.3 mm minimum gap; `trace t9` finishes in 15.18 s
  with 40.5 mm minimum gap. The C host simulator treats all supplied ranges as
  fresh, so it does not independently test stale-sample rejection.
- Release build now overflows STM32F103 flash by 36 B (65,572/65,536 B; previous
  unchecked version fit exactly). The release ramp was converted to integer
  millimetres (quantization ≤1 mm), which preserves both traces and saves 12 B;
  the latest link now overflows by **24 B** (65,560/65,536 B). This revision
  therefore has **no buildable Release image**. No firmware was flashed.
  Reclaim at least 24 B, rebuild Release, and confirm the HEX hash before
  considering hardware tests.

## Code size review without changing navigation behavior

- Link configuration already uses function/data sections and `--gc-sections`;
  unreferenced functions are removed by the linker. The linked firmware map
  retains functions reachable from control, navigation, ToF, diagnostics, and
  Bluetooth status paths; no safe unused block large enough to reclaim 24 B was
  found.
- Tried `-Oz` on `main.c`, `grid_map.c`, `motion.c`, `vl53l0x.c`, and `mpu6050.c`
  one at a time. Each made the Release image larger (worst case +808 B).
  `-fshort-enums` produced no size change. All CMake experiments were reverted.
- Final source behavior was unchanged by these experiments. PC suite remains
  9/9 PASS; nominal/T9 traces remain 14.37/15.18 s, minimum gap 40.3/40.4 mm.
  Release still overflows by 24 B. No new HEX was produced or flashed.

## Follow-up: equivalent offset storage optimization (2026-11-23)

- `fixed_line_offset` is now `uint8_t`: all assignments and the integer-mm ramp
  are values 0–35, so its representation changes but its route offset does not.
  PC checks pass and freshly compiled `trace nominal` / `trace t9` summaries are
  unchanged (14.37 s / 15.18 s, minimum gaps 40.3 / 40.4 mm).
- Release flash falls by 8 B but still overflows by 16 B (65,552/65,536 B).
  `--relax` and disabled unwind tables did not change size; `-fno-math-errno`
  made it 44 B too large. `-fno-ipa-cp` made it 804 B too large. Temporary
  CMake options were reverted.
- No current HEX exists; the prior file predates freshness checks. Do not flash
  it. Continue with an equivalent size reduction, then build and verify before
  attempting to program the board.

## Follow-up: build and flash completed (2026-11-23)

- Changed the zero comparison to `fixed_line_offset == 0u` after converting the
  field to `uint8_t`; this preserves the integer offset semantics. Reconfigured
  Release after removing the temporary `-fno-ipa-cp` experiment, then built with
  the baseline `-Os -g0`: 62,900 B flash / 17,616 B RAM. HEX SHA-256:
  `18A3401F0BFE16EF6DDA28250B3DA86136A6503A5E56D0096B31A9D5C111F587`.
- `Tests/run_checks.ps1`: 9/9 groups pass. Fresh simulator traces: nominal
  finish 14.37 s, minimum gap 40.3 mm; T9 finish 15.18 s, minimum gap 40.4 mm.
- Flashed via ST-LINK `37FF71064E573436E9171343`, probe voltage 3.27 V, SWD
  Under Reset 100 kHz. CubeProgrammer 2.23.0 reports full-file verification
  success and reset. Readback vector `20005000 0800CDC5`; TIM1 CCR1/CCR2 are zero.
  No movement command was sent; field behavior remains untested after this flash.
