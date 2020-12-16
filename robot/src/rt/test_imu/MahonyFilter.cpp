#include "orientation_tools.h"
#include "cppTypes.h"
#include <eigen3/Eigen/Dense>
#include <iostream>

typedef struct 
{
	float Kp;
	float Ki;
	float dt;

} PARAMS;


// void MahonyFilter(Quat<float>&, const Vec3<float>&, Vec3<float>&, Vec3<float>&, PARAMS&);

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


int main(int argc, char const *argv[])
{
	Quat<float> quat(.8, .1, .05, .02);
	Vec3<float> gyro(.05, .02, .01);
	Vec3<float> accel(.05, .02, .01);
	Vec3<float> err_integral(0, 0, 0);
	PARAMS params;

	params.Kp = 1.0;
	params.Ki = 0.0;
	params.dt = 0.001;

	MahonyFilter(quat, gyro, accel, err_integral, params);

	std::cout<< "quaternion " << std::endl << quat << std::endl 
			 << "gyro" <<  std::endl << gyro << std::endl 
			 << "accel" <<  std::endl << accel << std::endl;

	return 0;
}


