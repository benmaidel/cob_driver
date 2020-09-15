#include <cob_phidgets/phidget_motor_ros.h>

PhidgetMotorROS::PhidgetMotorROS(ros::NodeHandle nh, int serial_num, std::string board_name, XmlRpc::XmlRpcValue* sensor_params, SensingMode mode)
	:PhidgetMotor(mode), _nh(nh), _serial_num(serial_num), _board_name(board_name)
{
	ros::NodeHandle tmpHandle("~");
	ros::NodeHandle nodeHandle(tmpHandle, board_name);

	_pubAnalog = _nh.advertise<cob_phidgets::AnalogSensor>("analog_sensors", 1);
	_pubDigital = _nh.advertise<cob_phidgets::DigitalSensor>("digital_sensors", 1);
	_pubMotor = _nh.advertise<cob_phidgets::Motor>("motors", 1);
	_pubEncoder = _nh.advertise<cob_phidgets::Encoder>("encoders", 1);

	_subVelocity = _nh.subscribe("velocity", 1, &PhidgetMotorROS::onVelocityCallback, this);

	_srvSetAcceleration = nodeHandle.advertiseService("set_acceleration", &PhidgetMotorROS::setAccelerationCallback, this);
	_srvSetBackEMFSensingState = nodeHandle.advertiseService("set_back_emf_sensing_state", &PhidgetMotorROS::setBackEMFSensingStateCallback, this);
	_srvSetBraking = nodeHandle.advertiseService("set_braking", &PhidgetMotorROS::setBrakingCallback, this);
	_srvSetEncoderPosition = nodeHandle.advertiseService("set_encoder_position", &PhidgetMotorROS::setEncoderPositionCallback, this);

	if(init(_serial_num) != EPHIDGET_OK)
	{
		ROS_ERROR("Error open Phidget Board on serial %d. Message: %s",_serial_num, this->getErrorDescription(this->getError()).c_str());
	}
	if(waitForAttachment(10000) != EPHIDGET_OK)
	{
		ROS_ERROR("Error waiting for Attachment. Message: %s",this->getErrorDescription(this->getError()).c_str());
	}
	readParams(sensor_params);
}

PhidgetMotorROS::~PhidgetMotorROS()
{
}

auto PhidgetMotorROS::readParams(XmlRpc::XmlRpcValue* sensor_params) -> void
{
	if(sensor_params != nullptr)
	{
		for(auto& sensor : *sensor_params)
		{
			std::string name = sensor.first;
			XmlRpc::XmlRpcValue value = sensor.second;
			if(!value.hasMember("type"))
			{
				ROS_ERROR("Sensor Param '%s' has no 'type' member. Ignoring param!", name.c_str());
				continue;
			}
			if(!value.hasMember("index"))
			{
				ROS_ERROR("Sensor Param '%s' has no 'index' member. Ignoring param!", name.c_str());
				continue;
			}
			XmlRpc::XmlRpcValue value_type = value["type"];
			XmlRpc::XmlRpcValue value_index = value["index"];
			std::string type = value_type;
			int index = value_index;

			if(type == "analog")
			{
				_indexNameMapAnalog.insert(std::make_pair(index, name));
			}
			else if(type == "digital_in")
			{
				_indexNameMapDigital.insert(std::make_pair(index, name));
			}
			else if(type == "motor")
			{
				_indexNameMapMotor.insert(std::make_pair(index, name));
				if(value.hasMember("acceleration"))
				{
					XmlRpc::XmlRpcValue value_accel = value["acceleration"];
					double accel = value_accel;
					ROS_WARN("Setting acceleration to %f for sensor %s with index %d ", accel, name.c_str(), index);
					setAcceleration(index, accel);
				}
				if(value.hasMember("back_emf_sensing_mode"))
				{
					XmlRpc::XmlRpcValue value_state = value["back_emf_sensing_mode"];
					int state = value_state;
					ROS_WARN("Setting BackEMFSensingState to %d for sensor %s with index %d ", state, name.c_str(), index);
					setBackEMFSensingState(index, state);
				}
				if(value.hasMember("braking"))
				{
					XmlRpc::XmlRpcValue value_braking = value["braking"];
					double braking = value_braking;
					ROS_WARN("Setting braking to %f for sensor %s with index %d ", braking, name.c_str(), index);
					setBraking(index, braking);
				}
			}
			else if(type == "encoder")
			{
				_indexNameMapEncoder.insert(std::make_pair(index, name));

				if(value.hasMember("encoder_position"))
				{
					XmlRpc::XmlRpcValue value_position = value["encoder_position"];
					double position = value_position;
					ROS_WARN("Setting encoder_position to %d for sensor %s with index %d ", position, name.c_str(), index);
					setEncoderPosition(index, position);
				}
			}
			else
				ROS_ERROR("Type '%s' in sensor param '%s' is unkown. Ignoring sensor type.", type.c_str(), name.c_str());
		}
	}
	//fill up rest of maps with default values
	int count = this->getInputCount();
	for(int i = 0; i < count; i++)
	{
		_indexNameMapItr = _indexNameMapDigital.find(i);
		if(_indexNameMapItr == _indexNameMapDigital.end())
		{
			std::stringstream ss;
			ss << _board_name << "/" << i;
			_indexNameMapDigital.insert(std::make_pair(i, ss.str()));
		}
	}
	count = this->getSensorCount();
	for(int i = 0; i < count; i++)
	{
		_indexNameMapItr = _indexNameMapAnalog.find(i);
		if(_indexNameMapItr == _indexNameMapAnalog.end())
		{
			std::stringstream ss;
			ss << _board_name << "/" << i;
			_indexNameMapAnalog.insert(std::make_pair(i, ss.str()));
		}
	}

	count = this->getMotorCount();
	for(int i = 0; i < count; i++)
	{
		_indexNameMapItr = _indexNameMapMotor.find(i);
		if(_indexNameMapItr == _indexNameMapMotor.end())
		{
			std::stringstream ss;
			ss << _board_name << "/" << i;
			_indexNameMapMotor.insert(std::make_pair(i, ss.str()));
		}
	}

	count = this->getEncoderCount();
	for(int i = 0; i < count; i++)
	{
		_indexNameMapItr = _indexNameMapEncoder.find(i);
		if(_indexNameMapItr == _indexNameMapEncoder.end())
		{
			std::stringstream ss;
			ss << _board_name << "/" << i;
			_indexNameMapEncoder.insert(std::make_pair(i, ss.str()));
		}
	}


	//fill up reverse mapping
	count = this->getInputCount();
	for(int i = 0; i < count; i++)
	{
		_indexNameMapItr = _indexNameMapDigital.find(i);
		if(_indexNameMapItr != _indexNameMapDigital.end())
		{
			_indexNameMapDigitalRev.insert(std::make_pair(_indexNameMapDigital[i],i));
		}
	}
	count = this->getSensorCount();
	for(int i = 0; i < count; i++)
	{
		_indexNameMapItr = _indexNameMapAnalog.find(i);
		if(_indexNameMapItr != _indexNameMapAnalog.end())
		{
			_indexNameMapAnalogRev.insert(std::make_pair(_indexNameMapAnalog[i],i));
		}
	}
	count = this->getMotorCount();
	for(int i = 0; i < count; i++)
	{
		_indexNameMapItr = _indexNameMapMotor.find(i);
		if(_indexNameMapItr != _indexNameMapMotor.end())
		{
			_indexNameMapMotorRev.insert(std::make_pair(_indexNameMapMotor[i],i));
		}
	}
	count = this->getEncoderCount();
	for(int i = 0; i < count; i++)
	{
		_indexNameMapItr = _indexNameMapEncoder.find(i);
		if(_indexNameMapItr != _indexNameMapEncoder.end())
		{
			_indexNameMapEncoderRev.insert(std::make_pair(_indexNameMapEncoder[i],i));
		}
	}
}

auto PhidgetMotorROS::update() -> void
{
	int count = this->getInputCount();
	cob_phidgets::DigitalSensor msg_digit;
	std::vector<std::string> names;
	std::vector<signed char> states;

	//------- publish digital input states ----------//
	for(int i = 0; i < count; i++)
	{
		std::string name;
		_indexNameMapItr = _indexNameMapDigital.find(i);
		if(_indexNameMapItr != _indexNameMapDigital.end())
			name = (*_indexNameMapItr).second;
		names.push_back(name);
		states.push_back(this->getInputState(i));
	}
	msg_digit.header.stamp = ros::Time::now();
	msg_digit.uri = names;
	msg_digit.state = states;

	_pubDigital.publish(msg_digit);

	//------- publish analog input states ----------//
	cob_phidgets::AnalogSensor msg_analog;
	names.clear();
	std::vector<short int> values;
	count = this->getSensorCount();
	for(int i = 0; i < count; i++)
	{
		std::string name;
		_indexNameMapItr = _indexNameMapAnalog.find(i);
		if(_indexNameMapItr != _indexNameMapAnalog.end())
			name = (*_indexNameMapItr).second;
		names.push_back(name);
		values.push_back(this->getSensorValue(i));
	}
	msg_analog.header.stamp = ros::Time::now();
	msg_analog.uri = names;
	msg_analog.value = values;

	_pubAnalog.publish(msg_analog);

	//------- publish motor states ----------//
	cob_phidgets::Motor msg_motor;
	names.clear();
	std::vector<float> velocities;
	std::vector<float> accelerations;
	std::vector<float> currents;
	std::vector<int8_t> back_emf_sensing_states;
	std::vector<float> back_emfs;
	std::vector<float> brakings;
	count = this->getMotorCount();
	for(int i = 0; i < count; i++)
	{
		std::string name;
		_indexNameMapItr = _indexNameMapMotor.find(i);
		if(_indexNameMapItr != _indexNameMapMotor.end())
			name = (*_indexNameMapItr).second;
		names.push_back(name);
		velocities.push_back(this->getVelocity(i));
		accelerations.push_back(this->getAcceleration(i));
		currents.push_back(this->getCurrent(i));
		back_emf_sensing_states.push_back(this->getBackEMFSensingState(i));
		back_emfs.push_back(this->getBackEMF(i));
		brakings.push_back(this->getBraking(i));
	}
	msg_motor.header.stamp = ros::Time::now();
	msg_motor.uri = names;
	msg_motor.velocity = velocities;
	msg_motor.acceleration = accelerations;
	msg_motor.current = currents;
	msg_motor.back_emf_sensing_state = back_emf_sensing_states;
	msg_motor.back_emf = back_emfs;
	msg_motor.braking = brakings;

	_pubMotor.publish(msg_motor);

	//------- publish encoder states ----------//
	cob_phidgets::Encoder msg_encoder;
	names.clear();
	std::vector<int> positions;
	count = this->getEncoderCount();
	for(int i = 0; i < count; i++)
	{
		std::string name;
		_indexNameMapItr = _indexNameMapEncoder.find(i);
		if(_indexNameMapItr != _indexNameMapEncoder.end())
			name = (*_indexNameMapItr).second;
		names.push_back(name);
		positions.push_back(this->getEncoderPosition(i));
	}
	msg_encoder.header.stamp = ros::Time::now();
	msg_encoder.uri = names;
	msg_encoder.position = positions;

	_pubEncoder.publish(msg_analog);
}

auto PhidgetMotorROS::velocityChangeHandler(int index, double velocity) -> int
{
	ROS_DEBUG("Board %s: Velocity %d changed to value: %f", _board_name.c_str(), index, velocity);
}

auto PhidgetMotorROS::currentChangeHandler(int index, double current) -> int
{
	ROS_DEBUG("Board %s: Current %d changed to value: %f", _board_name.c_str(), index, current);
}
auto PhidgetMotorROS::inputChangeHandler(int index, int inputState) -> int
{
	ROS_DEBUG("Board %s: Input %d changed to value: %d", _board_name.c_str(), index, inputState);
}
auto PhidgetMotorROS::encoderPositionChangeHandler(int index, int time, int positionChange) -> int
{
	ROS_DEBUG("Board %s: EncoderPostion %d changed to value: %d", _board_name.c_str(), index, positionChange);
}
auto PhidgetMotorROS::encoderPositionUpdateHandler(int index, int positionChange) -> int
{
	ROS_DEBUG("Board %s: EncoderPostion %d updated to value: %d", _board_name.c_str(), index, positionChange);
}
auto PhidgetMotorROS::backEMFUpdateHandler(int index, double voltage) -> int
{
	ROS_DEBUG("Board %s: Back EMF %d updated to value: %f", _board_name.c_str(), index, voltage);
}
auto PhidgetMotorROS::sensorUpdateHandler(int index, int sensorValue) -> int
{
	ROS_DEBUG("Board %s: Sensor %d updated to value: %d", _board_name.c_str(), index, sensorValue);
}
auto PhidgetMotorROS::currentUpdateHandler(int index, double current) -> int
{
	ROS_DEBUG("Board %s: Current %d updated to value: %f", _board_name.c_str(), index, current);
}

auto PhidgetMotorROS::setAccelerationCallback(cob_phidgets::SetAcceleration::Request &req,
										cob_phidgets::SetAcceleration::Response &res) -> bool
{
	bool ret = false;

	// this check is nessesary because [] operator on Maps inserts an element if it is not found
	_indexNameMapRevItr = _indexNameMapMotorRev.find(req.uri);
	if(_indexNameMapRevItr != _indexNameMapMotorRev.end())
	{
		if(this->getAcceleration(_indexNameMapMotorRev[req.uri]) == req.acceleration)
		{
			ROS_INFO("Acceleration %i is already at value %f", _indexNameMapMotorRev[req.uri], req.acceleration);
			res.uri = req.uri;
			res.acceleration = req.acceleration;
			ret = true;
		}
		else
		{
			ROS_INFO("Setting acceleration %i to value %f", _indexNameMapMotorRev[req.uri], req.acceleration);
			this->setAcceleration(_indexNameMapMotorRev[req.uri], req.acceleration);

			res.uri = req.uri;
			res.acceleration = this->getAcceleration(_indexNameMapMotorRev[req.uri]);
			ret = true;
		}
	}
	else
	{
		ROS_DEBUG("Could not find uri '%s' inside port uri mapping", req.uri.c_str());
		res.uri = req.uri;
		res.acceleration = req.acceleration;
		ret = false;
	}

	return ret;
}

auto PhidgetMotorROS::setBackEMFSensingStateCallback(cob_phidgets::SetBackEMFSensingState::Request &req,
										cob_phidgets::SetBackEMFSensingState::Response &res) -> bool
{
	bool ret = false;

	// this check is nessesary because [] operator on Maps inserts an element if it is not found
	_indexNameMapRevItr = _indexNameMapMotorRev.find(req.uri);
	if(_indexNameMapRevItr != _indexNameMapMotorRev.end())
	{
		if(this->getBackEMFSensingState(_indexNameMapMotorRev[req.uri]) == req.back_emf_state)
		{
			ROS_INFO("Back EMF sensing state %i is already at value %d", _indexNameMapMotorRev[req.uri], req.back_emf_state);
			res.uri = req.uri;
			res.back_emf_state = req.back_emf_state;
			ret = true;
		}
		else
		{
			ROS_INFO("Setting back EMF sensing state %i to value %d", _indexNameMapMotorRev[req.uri], req.back_emf_state);
			this->setBackEMFSensingState(_indexNameMapMotorRev[req.uri], req.back_emf_state);

			res.uri = req.uri;
			res.back_emf_state = this->getBackEMFSensingState(_indexNameMapMotorRev[req.uri]);
			ret = true;
		}
	}
	else
	{
		ROS_DEBUG("Could not find uri '%s' inside port uri mapping", req.uri.c_str());
		res.uri = req.uri;
		res.back_emf_state = req.back_emf_state;
		ret = false;
	}

	return ret;
}

auto PhidgetMotorROS::setBrakingCallback(cob_phidgets::SetBraking::Request &req,
										cob_phidgets::SetBraking::Response &res) -> bool
{
	bool ret = false;

	// this check is nessesary because [] operator on Maps inserts an element if it is not found
	_indexNameMapRevItr = _indexNameMapMotorRev.find(req.uri);
	if(_indexNameMapRevItr != _indexNameMapMotorRev.end())
	{
		if(this->getBraking(_indexNameMapMotorRev[req.uri]) == req.braking)
		{
			ROS_INFO("Braking %i is already at value %f", _indexNameMapMotorRev[req.uri], req.braking);
			res.uri = req.uri;
			res.braking = req.braking;
			ret = true;
		}
		else
		{
			ROS_INFO("Setting braking %i to value %f", _indexNameMapMotorRev[req.uri], req.braking);
			this->setBraking(_indexNameMapMotorRev[req.uri], req.braking);

			res.uri = req.uri;
			res.braking = this->getBraking(_indexNameMapMotorRev[req.uri]);
			ret = true;
		}
	}
	else
	{
		ROS_DEBUG("Could not find uri '%s' inside port uri mapping", req.uri.c_str());
		res.uri = req.uri;
		res.braking = req.braking;
		ret = false;
	}

	return ret;
}

auto PhidgetMotorROS::setEncoderPositionCallback(cob_phidgets::SetEncoderPosition::Request &req,
										cob_phidgets::SetEncoderPosition::Response &res) -> bool
{
	bool ret = false;

	// this check is nessesary because [] operator on Maps inserts an element if it is not found
	_indexNameMapRevItr = _indexNameMapEncoderRev.find(req.uri);
	if(_indexNameMapRevItr != _indexNameMapEncoderRev.end())
	{
		if(this->getEncoderPosition(_indexNameMapEncoderRev[req.uri]) == req.encoder_position)
		{
			ROS_INFO("Encoder position %i is already at value %d", _indexNameMapEncoderRev[req.uri], req.encoder_position);
			res.uri = req.uri;
			res.encoder_position = req.encoder_position;
			ret = true;
		}
		else
		{
			ROS_INFO("Setting encoder position %i to value %d", _indexNameMapEncoderRev[req.uri], req.encoder_position);
			this->setEncoderPosition(_indexNameMapEncoderRev[req.uri], req.encoder_position);

			res.uri = req.uri;
			res.encoder_position = this->getEncoderPosition(_indexNameMapEncoderRev[req.uri]);
			ret = true;
		}
	}
	else
	{
		ROS_DEBUG("Could not find uri '%s' inside port uri mapping", req.uri.c_str());
		res.uri = req.uri;
		res.encoder_position = req.encoder_position;
		ret = false;
	}

	return ret;
}

auto PhidgetMotorROS::onVelocityCallback(const cob_phidgets::VelocityConstPtr& msg) -> void
{
	if(msg->uri.size() == msg->velocity.size())
	{
		for(size_t i = 0; i < msg->uri.size(); i++)
		{
			// this check is nessesary because [] operator on Maps inserts an element if it is not found
			_indexNameMapRevItr = _indexNameMapMotorRev.find(msg->uri[i]);
			if(_indexNameMapRevItr != _indexNameMapMotorRev.end())
			{
				ROS_DEBUG("Setting velocity of sensor %i to state %f", _indexNameMapMotorRev[msg->uri[i]], msg->velocity[i]);
				this->setVelocity(_indexNameMapMotorRev[msg->uri[i]], msg->velocity[i]);
			}
			else
				ROS_DEBUG("Could not find uri '%s' inside port uri mapping", msg->uri[i].c_str());
		}
	}
	else
	{
		ROS_ERROR("Received message with different uri and velocity container sizes");
	}
}

auto PhidgetMotorROS::attachHandler() -> int
{
	int serialNo, version, numInputs, numMotors, numEncoders, millis;
	int numSensors, triggerVal, ratiometric;
	const char *ptr, *name;

	CPhidget_getDeviceName((CPhidgetHandle)_motorHandle, &name);
	CPhidget_getDeviceType((CPhidgetHandle)_motorHandle, &ptr);
	CPhidget_getSerialNumber((CPhidgetHandle)_motorHandle, &serialNo);
	CPhidget_getDeviceVersion((CPhidgetHandle)_motorHandle, &version);

	CPhidgetMotorControl_getInputCount(_motorHandle, &numInputs);
	CPhidgetMotorControl_getMotorCount(_motorHandle, &numMotors);
	CPhidgetMotorControl_getEncoderCount(_motorHandle, &numEncoders);
	CPhidgetMotorControl_getSensorCount(_motorHandle, &numSensors);

	ROS_INFO("%s %d attached!!", name, serialNo);

	ROS_DEBUG("%s", ptr);
	ROS_DEBUG("Serial Number: %d\tVersion: %d", serialNo, version);
	ROS_DEBUG("Num Motors: %d", numMotors);
	ROS_DEBUG("Num Encoders: %d", numEncoders);
	ROS_DEBUG("Num Digital Inputs: %d", numInputs);
	ROS_DEBUG("Num Sensors: %d", numSensors);

	return 0;
}

auto PhidgetMotorROS::detachHandler() -> int
{
	int serial_number;
	const char *device_name;

	CPhidget_getDeviceName ((CPhidgetHandle)_motorHandle, &device_name);
	CPhidget_getSerialNumber((CPhidgetHandle)_motorHandle, &serial_number);
	ROS_INFO("%s Serial number %d detached!", device_name, serial_number);
	return 0;
}
