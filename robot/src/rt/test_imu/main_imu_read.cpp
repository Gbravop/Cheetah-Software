#include <iostream>

#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <termios.h>
#include <errno.h>
#include <math.h>
#include <pthread.h>

#include <endian.h>
#include <stdint.h>
#include "../../../include/rt/test_imu/config.h"
#include "../../../include/rt/test_imu/rt_imu.h"

using namespace std;

int main()
{

/**********************/
/**** init_serial ****/
/**********************/
init_serial();
/**********************/
/**** get_matches ****/
/**********************/

/**********************/
/****** get_fails *****/
/**********************/

/**********************/
/**** get_imu_data ****/
/**********************/
float* imu_d;
imu_d = get_imu_data();
cout << "IMU Data = "<< imu_d << endl;
/**********************/
/**** set_blocking ****/
/**********************/

/********************************/
/**** set_interface_attribs ****/
/******************************/


}
