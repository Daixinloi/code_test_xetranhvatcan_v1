# 2026-11-23 — ToF no-echo treated as clear on fixed waypoint route

## User report
The car reports a map/sensor error even though the lane is clear; user suspects an invalid ToF value is being interpreted as an obstacle/fault.

## Confirmed cause
`main.c` constructs `NavigationRange` with `.valid = s->ok && s->dist < 8000`. On a successful VL53L0X read with `8190` or `8191`, `.valid` is false but `.no_echo` is true (also requires `last_error == 0`). `FixedLineMove()` previously required `front[1].valid` unconditionally, so it failed `NAV_TOF_FAULT` on a clear no-return sample. This is not `NAV_SEARCH_BLOCKED` from map geometry.

## Change
`Src/navigation.c` now accepts a fresh range only when either it is valid, or it is explicitly flagged `no_echo` and equals sentinel `8190/8191`. Fixed-line center and diagonal freshness checks use this rule. A stale sample, a failed I2C read, an offline sensor, or any invalid sample without the no-echo flag still stops safely. Waypoints and avoidance geometry are unchanged.

## Checks
`Tests/run_checks.ps1` completed successfully: all 10 unit groups and 7 route traces passed.
- Nominal route with clear rays encoded as no-echo: finish.
- T9 side-obstacle route with clear center readings encoded as no-echo: finish in 16.05 s, modeled minimum clearance 40.1 mm.
- Clearable frontal obstacle: finish.
- `invalid_center`: rejected as `NAV_TOF_FAULT`, without vehicle motion.
- Large blocked frontal case: stops `NAV_SEARCH_BLOCKED`, without simulated contact.

## Build / flash
Release FLASH 64,308/65,536 B, RAM 17,648/20,480 B. HEX SHA-256 `095A24459DB6671BB11CF4255DBB7A357E7555BFC49547B94FAF0373212C102D`.
CubeProgrammer: STM32F103 medium-density 64 KB; ST-LINK SN `37FF71064E573436E9171343`; 3.49 V; `Download verified successfully`. Readback vector `0800D345`, TIM1 CCR1/CCR2 both zero. Vehicle behavior after this flash has not yet been observed.

## Next step
Run a supervised pass on the empty course. If it still faults, capture the exact navigation label and the three ToF values/status/ticks; only values `8190/8191` explicitly marked `no_echo` are treated as clear.
