/**
 * @file main_imu_test.cpp
 * @brief Code to schedule and control the execution of two threads:
 * (1) Read and retrieve IMU data (Thread 1),
 * (2) Compute quaternions from IMU gyro and acceleration data (Thread 2).
 * This code still needs to be integrated within HardwareBridge.cpp
 */
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

#include <endian.h>
#include <stdint.h>
#include "../../../include/rt/test_imu/rt_vector2nav.h"
#include "../../../include/rt/test_imu/config.h"
#include "../../../include/rt/test_imu/rt_imu.h"

#include "../../../include/rt/test_imu/orientation_tools.h"
#include "../../../include/rt/test_imu/cpp2Types.h"
#include <eigen3/Eigen/Dense>

#include <thread> // std::thread
#include <mutex>  // std::mutex, std::unique_lock
#include <condition_variable>

// using namespace std;

static std::mutex dataMutex, initMutex;

std::condition_variable _tcond1;
std::condition_variable _tcond2;

#define USE_KVH

typedef struct
{
	float Kp;
	float Ki;
	float dt;
} PARAMS;

void MahonyFilter(Quat<float>&quat, const Vec3<float>gyro, Vec3<float>&accel, Vec3<float>&err_integral, PARAMS&params){

	float Kp, Ki, dt;
	RotMat<float> world_R_body, body_R_world;
	Vec3<float> g_hat_in_body, err, gyro_corrected;
	Vec3<float> rpy;

	Kp = params.Kp;
	Ki = params.Ki;
	dt = params.dt;

	world_R_body = ori::quaternionToRotationMatrix(quat);
	body_R_world = world_R_body.transpose();

	accel = accel/accel.norm();
	g_hat_in_body = body_R_world.rightCols(1);

	err = accel.cross(g_hat_in_body);
	err_integral = err_integral + err*dt;

	gyro_corrected = gyro + Kp*err + Ki*err_integral;

	Quat<float> quatTemp(0, gyro_corrected(0), gyro_corrected(1), gyro_corrected(2));

	Quat<float> quaternion_dot = 0.5 * ori::quatProduct(quat, quatTemp);

	quat = quat + quaternion_dot * dt;

	quat = quat/quat.norm();

	rpy = ori::quatToRPY(quat);
}

void runKVH()
{
    // dataMutex.lock(); // Start of critical section
    init_serial(&dataMutex, &initMutex);
    // dataMutex.unlock(); // End of critical section
};

void processingKVH(){
    int quatc = 0; // Counter for quaternion computations - To be removed
    while (quatc<10000) {
        float* imu_d;
        dataMutex.lock(); // Start of critical section
        imu_d = get_imu_data();
        std::cout << "IMU Data = "<< imu_d << std::endl;
        /******************************/
        #ifdef USE_KVH
            Vec3<float> accel(imu_d[3], imu_d[4], imu_d[5]);
            Vec3<float> gyro(imu_d[0], imu_d[1], imu_d[2]);
        #endif
        dataMutex.unlock(); // End of critical section
        // Orientation Estimator:
        static Vec3<float> err_integral(0, 0, 0);
        static Quat<float> quat_real(0.8, 0.1, 0.5, 0.2);
        static Quat<float> quat_est(quat_real/quat_real.norm());
        // Filter Parameters:
        PARAMS params;
        params.Kp = 1.0;
        params.Ki = 0.0;
        params.dt = 1e-3;
        // Call to Filter Function:
        MahonyFilter(quat_est, gyro, accel, err_integral, params);
        // Output:
        Quat<float> quat(quat_est);

        std::cout<< "quaternion " << std::endl << quat << std::endl;
        std::cout << "counter_quat:"<< std::endl << quatc << std::endl;
        quatc++;
				usleep(1000);
    }
};

int main()
{
    // Read IMU data
		initMutex.lock();
    std::thread KVH_READ(runKVH);
    // Compute quaternions
		initMutex.lock();
		std::cout << "Starting Pro\n";
    std::thread KVH_COMPUTE(processingKVH);

    KVH_READ.join();
    KVH_COMPUTE.join();
    return 0;
}
