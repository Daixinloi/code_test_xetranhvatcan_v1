#ifndef TEACH_COURSE_H
#define TEACH_COURSE_H

#include "teach_storage.h"

/* Fixed obstacle layout, starting at (420, -45) mm facing north.
 * Stop and pivot at the corners, especially before heading south from T14.
 * F targets use 0.1265 mm/E2 count. Turn targets are in tenths of a degree. */
static const TeachSegment teach_course[] = {
    {5415, 'F', 0}, /* (420, 640) mm */
    { 283, 'R', 0},
    {1167, 'F', 0}, /* (490, 770) mm */
    { 216, 'L', 0},
    { 876, 'F', 0}, /* (503, 880) mm */
    { 839, 'R', 0},
    {3574, 'F', 0}, /* (955, 875) mm */
    { 411, 'R', 0},
    {2085, 'F', 0}, /* (1152, 700) mm */
    { 418, 'L', 0},
    {1171, 'F', 0}, /* (1300, 700) mm, below T14 */
    { 901, 'R', 0},
    {6048, 'F', 0}, /* (1300, -65) mm, entire body across the finish */
};

#endif
