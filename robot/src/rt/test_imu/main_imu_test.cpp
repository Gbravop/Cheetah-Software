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
#include "../../../include/rt/test_imu/rt_vector2nav.h"
#include "../../../include/rt/test_imu/config.h"
#include "../../../include/rt/test_imu/rt_imu.h"

#include "../../../include/rt/test_imu/orientation_tools.h"
#include "../../../include/rt/test_imu/cpp2Types.h"
#include <eigen3/Eigen/Dense>

#include <thread>
#include <mutex>
#include <condition_variable>

using namespace std;

condition_variable _tcond1;
condition_variable _tcond2;

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

class Thread1{
private:
  mutex _lockprint;
  bool isThreadAlive = true;

public:
  Thread1(){}
  Thread1(Thread1 &st){};

  void StartProcessing()
  {
    unique_lock<<mutex>> locker(_lockprint);
    init_serial();
    _tcond1.wait(locker);
    cout << "Thread1" << endl;
    _tcond2.notify_one();
  }
  void operator()()
  {
    while (isThreadAlive)
    StartProcessing();
  }

  void stopThread()
  {
    isThreadAlive = false;
  }
};

class Thread2{
private:
  std::mutex _lockprint;
  bool isThreadAlive = true;

public:
  Thread2(){}
  Thread2(Thread2 &st){};

  void StartProcessing(){
    unique_lock<<mutex>> locker(_lockprint);
    int quatc = 0;
    while (quatc<2) {
      float* imu_d;
      imu_d = get_imu_data();
      cout << "IMU Data = "<< imu_d << endl;
      /******************************/
      #ifdef USE_KVH
        Vec3<float> accel(imu_d[3], imu_d[4], imu_d[5]);
        Vec3<float> gyro(imu_d[0], imu_d[1], imu_d[2]);
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
      #endif
      cout << "counter_quat:"<< quatc << '\n';
      quatc++;
    }
    _tcond2.wait(locker);
    cout << "Thread2" << endl;
    _tcond1.notify_one();
  }

  void operator()()
  {
    while (isThreadAlive)
    StartProcessing();
  }

  void stopThread()
  {
    isThreadAlive = false;
  }
};

int main()
{
  Thread1 st1;
  Thread2 st2;
  thread t1(st1);
  thread t2(st2)
  _tcond1.notify_one();
  t1.detach();
  t2.detach();
  Sleep(10);
  st1.stopThread();
  st2.stopThread();
  return 0;
  }
