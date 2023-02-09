#include "vrep_bridge.h"

VRepBridge::VRepBridge(ControlMode mode)
{
	control_mode_ = mode;
	simInit();
	getHandle();
	desired_torque_.setZero();
}
VRepBridge::~VRepBridge()
{
	simxStopSimulation(clientID_, simx_opmode_oneshot_wait);
	simxFinish(clientID_);
}

bool VRepBridge::simConnectionCheck()
{
	return (simxGetConnectionId(clientID_) != -1);
}
void VRepBridge::simLoop()
{
	traj_tick_++;
	tick_++;
	simxSynchronousTrigger(clientID_);
}
void VRepBridge::simxErrorCheck(simxInt error)
{
	string errorMsg;
	switch (error)
	{
	case simx_error_noerror:
		return;	// no error
		break;
	case simx_error_timeout_flag:
		errorMsg = "The function timed out (probably the network is down or too slow)";
		break;
	case simx_error_illegal_opmode_flag:
		errorMsg = "The specified operation mode is not supported for the given function";
		break;
	case simx_error_remote_error_flag:
		errorMsg = "The function caused an error on the server side (e.g. an invalid handle was specified)";
		break;
	case simx_error_split_progress_flag:
		errorMsg = "The communication thread is still processing previous split command of the same type";
		break;
	case simx_error_local_error_flag:
		errorMsg = "The function caused an error on the client side";
		break;
	case simx_error_initialize_error_flag:
		errorMsg = "simxStart was not yet called";
		break;
	default:
		errorMsg = "Unknown error.";
		break;
	}

	cout << "[ERROR] An error is occured. code = " << error << endl;
	cout << " - Description" << endl;
	cout << " | " << errorMsg << endl;

	throw std::string(errorMsg);
}

void VRepBridge::simInit()
{
	simxFinish(-1);
	clientID_ = simxStart("127.0.0.1", -3, true, true, 2000, 5);
	if (clientID_ < 0)
	{
		throw std::string("Failed connecting to remote API server. Exiting.");
	}

	simxErrorCheck(simxStartSimulation(clientID_, simx_opmode_oneshot_wait));
	simxErrorCheck(simxSynchronous(clientID_, true));

	cout << "[INFO] V-Rep connection is established." << endl;

}

void VRepBridge::write()
{
	switch (control_mode_)
	{
	case CTRL_POSITION:
	{
		for (size_t i = 0; i < DOF; i++)
		{
			simxSetJointTargetPosition(clientID_, motorHandle_[i], desired_q_(i), simx_opmode_streaming);
		}
		simxSetJointTargetPosition(clientID_, motorHandle_ee_[0], desired_ee_(0), simx_opmode_streaming);
		simxSetJointTargetPosition(clientID_, motorHandle_ee_[1], desired_ee_(1), simx_opmode_streaming);
	}
	}
}
void VRepBridge::read()
{
	for (size_t i = 0; i < DOF; i++)
	{    
		simxFloat data;
		simxGetJointPosition(clientID_, motorHandle_[i], &data, simx_opmode_streaming);
		current_q_(i) = data;
		simxGetJointForce(clientID_, motorHandle_[i], &data, simx_opmode_streaming);
		current_torque_(i) = data;
		simxGetObjectFloatParameter(clientID_, motorHandle_[i], 2012, &data, simx_opmode_streaming);
		current_q_dot_(i) = data;
	}
	for (size_t i = 0; i < 2; i++)
	{  
		simxFloat data;
		simxGetJointPosition(clientID_, motorHandle_ee_[i], &data, simx_opmode_streaming);
		current_ee_(i) = data;
		simxGetObjectFloatParameter(clientID_, motorHandle_ee_[i], 2012, &data, simx_opmode_streaming);
		current_ee_dot_(i) = data;
	}

	float force[3];
	float torque[3];
	simxUChar state;
	simxReadForceSensor(clientID_, motorHandle_ee_[2], &state, force, torque, simx_opmode_streaming);
	for (int i = 0; i < 3; i++)
	{
		dataForce_[i] = force[i];
		dataTorque_[i] = torque[i];
	}
}

void VRepBridge::setDesiredPosition(const Eigen::Matrix<double, DOF, 1>& desired_q)
{
	desired_q_ = desired_q;
}

void VRepBridge::setDesiredEndEffectorPosition(const Eigen::Matrix<double, 2, 1>& desired_ee)
{
	desired_ee_ = desired_ee;
}

void VRepBridge::setDesiredTorque(const Eigen::Matrix<double, DOF, 1>& desired_torque)
{
	desired_torque_ = desired_torque;
}

const Eigen::Matrix<double, DOF, 1>& VRepBridge::getPosition()
{
	return current_q_;
}

const Eigen::Matrix<double, DOF, 1>& VRepBridge::getVelocity()
{
	return current_q_dot_;
}


const Eigen::Matrix<double, 2, 1>& VRepBridge::getPosition_ee()
{
	return current_ee_;
}

const Eigen::Matrix<double, 2, 1>& VRepBridge::getVelocity_ee()
{
	return current_ee_dot_;
}

const Eigen::Matrix<double, DOF, 1>& VRepBridge::getTorque()
{
	return current_torque_;
}
const Eigen::Matrix<double, 3, 1>& VRepBridge::getTorque_ee()
{
	return dataTorque_;
}

const Eigen::Matrix<double, 3, 1>& VRepBridge::getForce_ee()
{
	return dataForce_;
}

void VRepBridge::getHandle()
{
	cout << "[INFO] Getting handles." << endl;
	const string joint_name1 = JOINT_HANDLE_PREFIX_1;
	const string joint_name2 = JOINT_HANDLE_PREFIX_2;
	const string joint_name3 = JOINT_HANDLE_PREFIX_3;
	simxErrorCheck(simxGetObjectHandle(clientID_, joint_name1.c_str(), &motorHandle_ee_[0], simx_opmode_oneshot_wait));
	cout << "[INFO] Getting a handle named " << joint_name1 << endl;
	simxErrorCheck(simxGetObjectHandle(clientID_, joint_name2.c_str(), &motorHandle_ee_[1], simx_opmode_oneshot_wait));
	cout << "[INFO] Getting a handle named " << joint_name2 << endl;
	simxErrorCheck(simxGetObjectHandle(clientID_, joint_name3.c_str(), &motorHandle_ee_[2], simx_opmode_oneshot_wait));
	cout << "[INFO] Getting a handle named " << joint_name3 << endl;
		

	for (int i = 0; i < DOF; i++)
	{
		const string joint_name = JOINT_HANDLE_PREFIX + std::to_string(i + 1);
		cout << "[INFO] Getting a handle named " << joint_name << endl;
		simxErrorCheck(simxGetObjectHandle(clientID_, joint_name.c_str(), &motorHandle_[i], simx_opmode_oneshot_wait));
	}

	cout << "[INFO] The handle has been imported." << endl;
}
