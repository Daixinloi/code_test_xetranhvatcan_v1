"""Check the front-edge avoidance layer against the real obstacle table.

Models the firmware's own sensor geometry (NavigationConfig.sensor_forward_mm,
sensor_left_mm, sensor_angle_deg) and the two walls of the finish corridor:

  * middle exclusion {600,0,1200,600}  -> right edge X = 1200, Y 0..600
  * T15 {1400,400,1500,500}            -> left face  X = 1400, Y 400..500

The layer in Src/navigation.c reads front[2] (right beam, -25 deg) while the
body descends south and steers the aim so that the reported range stays at
FINISH_BEAM_MM.  This script converts body centre X -> beam range and back so
the equilibrium and the clearances can be checked without the car.

Run:  python runs/20261122-front-edge-avoid/validate_beam.py
"""

import math

U_RIGHT = 1200.0          # middle exclusion right edge
U_Y0, U_Y1 = 0.0, 600.0   # middle exclusion Y span
T15_LEFT = 1400.0         # T15 left face
T15_Y0, T15_Y1 = 400.0, 500.0
BODY_HALF = 45.0          # 90 x 90 mm body
FORWARD = 45.0            # sensor_forward_mm[2]
LATERAL = -25.0           # sensor_left_mm[2]  (negative = right of centre)
ANGLE_DEG = -25.0         # sensor_angle_deg[2]
BEAM_MM = 118.0           # FINISH_BEAM_MM in Src/navigation.c
LANE_MIN_X = 1255.0       # 1200 + 45 mm half-body + 10 mm margin
LANE_MAX_X = 1345.0       # 1400 - 45 mm half-body - 10 mm margin
GAIN_SIN = math.sin(math.radians(25.0))
YAW = -90.0               # south, degrees

COS_YAW = math.cos(math.radians(YAW))
SIN_YAW = math.sin(math.radians(YAW))
RAY = math.radians(YAW + ANGLE_DEG)
RAY_DX, RAY_DY = math.cos(RAY), math.sin(RAY)


def beam_origin(x, y):
    """Right beam origin, same expression as Navigation_ProjectRange()."""
    return (x + FORWARD * COS_YAW - LATERAL * SIN_YAW,
            y + FORWARD * SIN_YAW + LATERAL * COS_YAW)


def range_to_u_edge(x, y):
    """Slant range from the right beam to the U block right edge, or None."""
    ox, oy = beam_origin(x, y)
    if ox <= U_RIGHT:
        return None                      # beam points away from the block
    t = (U_RIGHT - ox) / RAY_DX
    if t <= 0.0:
        return None
    hit_y = oy + t * RAY_DY
    if not (U_Y0 <= hit_y <= U_Y1):
        return None                      # beam passes under the block
    return t


def centre_from_range(r):
    """Inverse of range_to_u_edge at Y = 500 (hit_y inside the block)."""
    ox = U_RIGHT - r * RAY_DX
    return ox - FORWARD * COS_YAW + LATERAL * SIN_YAW


def layer_aim(x, y, aim_held):
    """Return (aim, held) exactly as the firmware layer does."""
    r = range_to_u_edge(x, y)
    if r is None or r < 40.0 or r > 480.0:
        return aim_held, True             # beam lost the block: hold last aim
    return (BEAM_MM - r) * GAIN_SIN, True


def corrected_target(x, y, aim_held):
    aim, held = layer_aim(x, y, aim_held)
    return min(LANE_MAX_X, max(LANE_MIN_X, x + aim)), aim, held


def clearances(x):
    return (x - BODY_HALF - U_RIGHT, T15_LEFT - (x + BODY_HALF))


def main():
    print("layer setpoint FINISH_BEAM_MM =", BEAM_MM, "mm")
    print("beam range at X=1280, Y=500  =", round(range_to_u_edge(1280.0, 500.0), 1), "mm")
    print("centre X for that range      =", round(centre_from_range(BEAM_MM), 1), "mm")
    print()

    print("X     range   aim(off)  U-gap  T15-gap   note")
    for x in (1230.0, 1250.0, 1280.0, 1300.0, 1330.0, 1350.0, 1390.0):
        r = range_to_u_edge(x, 500.0)
        if r is None:
            print(f"{x:6.1f}  lost     -         -      -        beam misses the block")
            continue
        aim = (BEAM_MM - r) * GAIN_SIN
        u_gap, t_gap = clearances(x)
        print(f"{x:6.1f}  {r:6.1f}  {aim:7.1f}  {u_gap:6.1f}  {t_gap:6.1f}")

    print()
    print("closed loop: aim = X + aim_off, body moves 40% of the error per tick")
    for drift in (0.0, 88.0, 120.0):
        x = 1302.0 + drift      # true centre at the top of the final leg
        aim = 0.0
        for _ in range(200):
            target, aim, _ = corrected_target(x, 500.0, aim)
            x += 0.4 * (target - x)
        u_gap, t_gap = clearances(x)
        print(f"  drift {drift:5.1f} mm -> settles at X={x:6.1f}  "
              f"U gap {u_gap:5.1f} mm  T15 gap {t_gap:5.1f} mm")

    print("\ncorrected target envelope for valid beam ranges:")
    for x in (1200.0, 1230.0, 1280.0, 1390.0, 1450.0):
        for y in (500.0,):
            target, _, _ = corrected_target(x, y, 0.0)
            assert LANE_MIN_X <= target <= LANE_MAX_X
            print(f"  body X={x:6.1f} -> target X={target:6.1f} mm")

    print()
    print("beam reach on the descent (Y window where the right beam sees the block):")
    lo = hi = None
    for y in range(-60, 660):
        if range_to_u_edge(1280.0, float(y)) is not None:
            lo = y if lo is None else lo
            hi = y
    print(f"  X=1280 -> Y {lo} .. {hi} mm, so the layer holds its last aim below Y={lo}")


if __name__ == "__main__":
    main()
