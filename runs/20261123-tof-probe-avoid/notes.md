# 2026-11-23 — Reactive ToF probe at waypoint T9 corridor

## Objective
Keep the hand-tuned waypoint course as the nominal path. Add a local ToF response when the centre sensor sees a frontal obstruction, while preserving the diagonal-beam response that handles the sideways graze at T9.

## Firmware changes
- `Src/navigation.c`: retain waypoint list and the established diagonal trigger (centre >=300 mm and one diagonal <=100 mm; current offset 40 mm, hold 400 mm, release 120 mm).
- Fixed-line mode now checks centre ToF freshness and eases cruise below 80 mm. Two distinct centre samples below 30 mm start a perpendicular 80 mm probe, right first. It turns back to the course heading and accepts the shifted course only when centre plus both diagonal readings are fresh and >50 mm. It tries the opposite side if the current side does not look clear; map rejection or blocked readings stop the car safely.
- `Tests/sim_course.c`: added a centered frontal-obstacle scenario and assertions for nominal, T9, clearable frontal obstacle, and blocked frontal obstacle. `Tests/run_checks.ps1` builds and runs those four scenarios.
- `COURSE_ROUTE` waypoint data was not changed.

## PC checks
`Tests/run_checks.ps1`: all 10 unit groups PASS; added four simulation trace scenarios exit successfully.

- `trace nominal`: finish, 14.37 s, minimum modeled clearance 40.3 mm.
- `trace t9`: finish, 16.05 s, minimum modeled clearance 40.1 mm.
- `trace front_open`: a 40×40 mm box at x=990..1030, y=840..880 mm is bypassed; finish in 17.68 s, minimum modeled clearance 37.3 mm.
- `trace extra`: 100×100 mm box at x=990..1090, y=800..900 mm leaves no accepted local pass in this corridor; the controller stops in `NAV_SEARCH_BLOCKED` near x=913.4, y=785.4 mm. No simulator contact occurred. This is a safe stop, not a successful bypass.

## Build and flash
Release build succeeded: FLASH 64,260/65,536 B, RAM 17,648/20,480 B. HEX SHA-256 `B9F8E6C2712D730DFF5E2040C325C4BD16FB61B5ED3207F7B8CB98B7DBC8E373`.
CubeProgrammer reported `Download verified successfully`; observed target STM32F103 medium-density, 64 KB, voltage 3.26–3.27 V. Readback vector at `0x08000004`: `0800D315`; TIM1 CCR1 and CCR2 read zero after reset. `-l` listed the single probe SN `37FF71064E573436E9171343`; one programming log printed the USB instance string instead of SN, so this is documented. No vehicle run has verified the behavior yet.

## Next
Supervised physical test near T9 and with a small frontal obstacle; keep a hand on the stop command. Capture pose/yaw and all three ToF ranges if it stops or brushes a box. Do not infer real-world success from the simulator.
