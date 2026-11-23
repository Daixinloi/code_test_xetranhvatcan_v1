#ifndef NAVIGATION_CONFIG_H
#define NAVIGATION_CONFIG_H
#include "navigation.h"

/* Supervised field-test configuration, based on the user's dimensions/photo.
 * See NAVIGATION_SETUP.md; all distances are millimetres, not centimetres. */
static const NavigationConfig navigation_config = {
    .enabled = true,
    .route_u = true,
    .length_mm = 90, .width_mm = 90,
    /* The U route applies a separate 50 mm margin around the middle exclusion.
     * Body collision geometry retains an extra 10 mm nominal margin. */
    .clearance_mm = 0, .position_margin_mm = 10,
#ifdef NAV_TEST_START_X_MM
    .start_x_mm = NAV_TEST_START_X_MM,
#else
    .start_x_mm = 300, /* User-selected X = 30 cm. */
#endif
    .start_y_mm = -45, /* Nose on start line; centre is 4.5 cm outside. */
    .goal_x_mm = 1575, .goal_y_mm = 125,
    .start_heading_deg = 90, /* User-confirmed north. */
    .sensor_forward_mm = {45, 45, 45},
    .sensor_left_mm = {25, 0, -25}, /* Estimated from photo; adjust if measured. */
    .sensor_angle_deg = {25, 0, -25} /* User-confirmed relative to centre. */
};
#endif
