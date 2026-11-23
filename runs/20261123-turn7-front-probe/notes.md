# 2026-11-23 — 7 degree tap turns and 25 mm frontal ToF probe

## User report / requested behavior
The waypoint route has now reached the finish. Turns feel slow; one tap should turn 7 degrees instead of 5. A frontal obstacle should cause gradual slowdown, then a perpendicular search at about 2.5 cm.

## Changes
- `Src/teach_drive.c`: `TAP_TURN_TENTHS` 50 → 70, changing an unheld R/L tap from 5° to 7°. Continuous held turn and saved route replay are unchanged. This is part of the Teach preset only.
- `Src/navigation.c`: fixed-line center trigger 30 → 25 mm. Existing perpendicular probe remains 80 mm, right side first, then turns to route heading and requires fresh center and both diagonals >50 mm before resuming. Approach speed now ramps from 210 mm/s at 80 mm down to 90 mm/s at 25 mm; the probe runs at 90 mm/s.
- Waypoint table unchanged. Current active Release retains automatic navigation and ToF; Teach image was built for verification but not flashed.

## Checks
`Tests/run_checks.ps1`: 10 unit groups and 7 simulator traces passed, including exact 7° tap stop, no-echo behavior, invalid center fault, perpendicular frontal probe and safe blocked stop.
- nominal: finish, 14.37 s, 40.3 mm minimum modeled clearance.
- t9: finish, 16.05 s, 40.1 mm.
- front_open: 40×40 mm centered box triggers slow approach and perpendicular probe, finish 17.73 s, 36.2 mm.
- A large box in the narrow corridor still produces safe stop; simulator evidence does not prove all 100×100 mm placements are passable.

## Build / flash
Release: FLASH 64,312/65,536 B; RAM 17,648/20,480 B. HEX SHA-256 `D60D399C824148ED89B88E5237E88428F64E76BE4199380355F59CE4712509E9`.
Teach: build succeeded; HEX SHA-256 `6BAB5AA6163BAAB57BF75036C4BB4FE8FD3288A18397B6EA5EA98D0F4157467E`; not flashed.
Release was flashed and CubeProgrammer reported `Download verified successfully`. STM32F103 medium density, 64 KB; ST-LINK `37FF71064E573436E9171343`; 3.27 V; vector readback `0800D349`; TIM1 CCR1/CCR2 both zero. Vehicle not yet driven after this flash.

## Next
Supervised on-course test with stop command ready. Verify the actual target is the Release build if using automatic waypoints. For a physical frontal obstacle, record all three ToF readings and the Navigation status if it stops.
