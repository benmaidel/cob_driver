#ifndef _PHIDGET_MOTOR_ROS_H_
#define _PHIDGET_MOTOR_ROS_H_

#include <cob_phidgets/phidget_motor.h>

#include <ros/ros.h>

#include <cob_phidgets/AnalogSensor.h>
#include <cob_phidgets/DigitalSensor.h>
#include <cob_phidgets/Encoder.h>
#include <cob_phidgets/Motor.h>
#include <cob_phidgets/Velocity.h>

#include <cob_phidgets/SetAcceleration.h>
#include <cob_phidgets/SetBackEMFSensingState.h>
#include <cob_phidgets/SetBraking.h>
#include <cob_phidgets/SetEncoderPosition.h>

#include <thread>
#include <mutex>
#include <map>

class PhidgetMotorROS: public PhidgetMotor
{
public:
	PhidgetMotorROS(ros::NodeHandle nh, int serial_num, std::string board_name, XmlRpc::XmlRpcValue* sensor_params, SensingMode mode);
	~PhidgetMotorROS();

private:
	ros::NodeHandle _nh;
	ros::Publisher _pubMotor;
	ros::Publisher _pubDigital;
	ros::Publisher _pubAnalog;
	ros::Publisher _pubEncoder;
	ros::Subscriber _subVelocity;
	ros::ServiceServer _srvSetAcceleration;
	ros::ServiceServer _srvSetBackEMFSensingState;
	ros::ServiceServer _srvSetBraking;
	ros::ServiceServer _srvSetEncoderPosition;

	int _serial_num;
	std::string _board_name;

	std::mutex _mutex;

	std::map<int, std::string> _indexNameMapAnalog;
	std::map<std::string, int> _indexNameMapAnalogRev;
	std::map<int, std::string> _indexNameMapDigital;
	std::map<std::string, int> _indexNameMapDigitalRev;
	std::map<int, std::string> _indexNameMapEncoder;
	std::map<std::string, int> _indexNameMapEncoderRev;
	std::map<int, std::string> _indexNameMapMotor;
	std::map<std::string, int> _indexNameMapMotorRev;

	std::map<int, std::string>::iterator _indexNameMapItr;
	std::map<std::string, int>::iterator _indexNameMapRevItr;

	auto readParams(XmlRpc::XmlRpcValue* sensor_params) -> void;

	auto update() -> void;

	auto attachHandler() -> int;
	auto detachHandler() -> int;

	auto velocityChangeHandler(int index, double velocity) -> int;
	auto currentChangeHandler(int index, double current) -> int;
	auto inputChangeHandler(int index, int inputState) -> int;
	auto encoderPositionChangeHandler(int index, int time, int positionChange) -> int;
	auto encoderPositionUpdateHandler(int index, int positionChange) -> int;
	auto backEMFUpdateHandler(int index, double voltage) -> int;
	auto sensorUpdateHandler(int index, int sensorValue) -> int;
	auto currentUpdateHandler(int index, double current) -> int;


	auto onVelocityCallback(const cob_phidgets::VelocityConstPtr& msg) -> void;

	auto setAccelerationCallback(cob_phidgets::SetAcceleration::Request &req,
									cob_phidgets::SetAcceleration::Response &res) -> bool;

	auto setBackEMFSensingStateCallback(cob_phidgets::SetBackEMFSensingState::Request &req,
										cob_phidgets::SetBackEMFSensingState::Response &res) -> bool;

	auto setBrakingCallback(cob_phidgets::SetBraking::Request &req,
										cob_phidgets::SetBraking::Response &res) -> bool;

	auto setEncoderPositionCallback(cob_phidgets::SetEncoderPosition::Request &req,
										cob_phidgets::SetEncoderPosition::Response &res) -> bool;
};
#endif //_PHIDGETIK_H_
