/*!
 * @file rt_vectornav.h
 * @brief VectorNav IMU communication
 */

#ifndef _rt_vectornav
#define _rt_vectornav

#ifdef linux

#include <lcm/lcm-cpp.hpp>
#include "IMU2Types.h"

#ifdef __cplusplus
extern "C" {
#endif

// #include "vn/sensors.h"
//
// #ifdef __cplusplus
// }
// #endif

bool init_vectornav(VectorNavData* vd_data);

#endif
#endif
