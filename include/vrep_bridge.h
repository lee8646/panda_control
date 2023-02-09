#pragma once

#include <iostream>
#include <string>
#include <functional>
#include <Eigen/Dense>

using namespace std;

extern "C" {
#include "extApi.h"
}

const std::string JOINT_HANDLE_PREFIX{ "panda_joint" };
const std::string JOINT_HANDLE_PREFIX_1{ "panda_finger_joint1" };
const std::string JOINT_HANDLE_PREFIX_2{ "panda_finger_joint2" };
const std::string JOINT_HANDLE_PREFIX_3{ "Force_sensor" };


class VRepBridge
{
public:
	enum ControlMode { CTRL_POSITION, CTRL_VELOCITY, CTRL_TORQUE };

	VRepBridge(ControlMode mode = CTRL_POSITION);
	~VRepBridge();
	
	bool simConnectionCheck();
	void simLoop();

	void write();
	void read();

	void setDesiredPosition(const Eigen::Matrix<double, DOF, 1> & desired_q);
	void setDesiredEndEffectorPosition(const Eigen::Matrix<double, 2, 1> & desired_ee_);
	void setDesiredTorque(const Eigen::Matrix<double, DOF, 1> & desired_torque);
	const Eigen::Matrix<double, DOF, 1> & getPosition();
	const Eigen::Matrix<double, DOF, 1> & getVelocity();
	const Eigen::Matrix<double, 2, 1> & getPosition_ee();
	const Eigen::Matrix<double, 2, 1> & getVelocity_ee();
	const Eigen::Matrix<double, DOF, 1> & getTorque();
	const Eigen::Matrix<double, 3, 1> & getTorque_ee();
	const Eigen::Matrix<double, 3, 1> & getForce_ee();

	const size_t getTick() { return tick_; }

private:
	Eigen::Matrix<double, DOF, 1> current_q_;
	Eigen::Matrix<double, DOF, 1> current_q_dot_;
    Eigen::Matrix<double, DOF, 1> current_torque_;
	Eigen::Matrix<double, DOF, 1> desired_q_;
	Eigen::Matrix<double, DOF, 1> desired_torque_;
	Eigen::Matrix<double, 2, 1> current_ee_;
	Eigen::Matrix<double, 2, 1> current_ee_dot_;
	Eigen::Matrix<double, 2, 1> desired_ee_;
    Eigen::Matrix<double, 3, 1> dataForce_;
	Eigen::Matrix<double, 3, 1> dataTorque_;

	simxInt clientID_;
	simxInt motorHandle_[7];
	simxInt motorHandle_ee_[3];	/// < Depends on simulation envrionment
	simxInt objectHandle_;

	size_t tick_{ 0 };
	size_t traj_tick_{ 0 };
	
	ControlMode control_mode_;

	void simxErrorCheck(simxInt error);
	void simInit();
	void getHandle(); 	/// < Depends on simulation envrionment
};
