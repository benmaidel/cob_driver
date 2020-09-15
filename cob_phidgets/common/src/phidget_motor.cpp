#include <cob_phidgets/phidget_motor.h>

PhidgetMotor::PhidgetMotor(SensingMode mode)
: Phidget((CPhidgetHandle*) &_motorHandle, mode), _motorHandle(0)
{
	_last_error = CPhidgetMotorControl_create(&_motorHandle);

	if (!_last_error) {
		CPhidget_set_OnAttach_Handler((CPhidgetHandle) _motorHandle,
				PhidgetMotor::attachDelegate, this);

		CPhidget_set_OnDetach_Handler((CPhidgetHandle) _motorHandle,
				PhidgetMotor::detachDelegate, this);

		if(_sensMode == SensingMode::EVENT)
		{
			_last_error = CPhidgetMotorControl_set_OnVelocityChange_Handler(_motorHandle,
				PhidgetMotor::velocityChangeDelegate, this);
			
			_last_error = CPhidgetMotorControl_set_OnCurrentChange_Handler(_motorHandle,
				PhidgetMotor::currentChangeDelegate, this);

			_last_error = CPhidgetMotorControl_set_OnInputChange_Handler(_motorHandle,
				PhidgetMotor::inputChangeDelegate, this);

			_last_error = CPhidgetMotorControl_set_OnEncoderPositionChange_Handler(_motorHandle,
				PhidgetMotor::encoderPositionChangeDelegate, this);

			_last_error = CPhidgetMotorControl_set_OnEncoderPositionUpdate_Handler(_motorHandle,
				PhidgetMotor::encoderPositionUpdateDelegate, this);

			_last_error = CPhidgetMotorControl_set_OnBackEMFUpdate_Handler(_motorHandle,
				PhidgetMotor::backEMFUpdateDelegate, this);

			_last_error = CPhidgetMotorControl_set_OnSensorUpdate_Handler(_motorHandle,
				PhidgetMotor::sensorUpdateDelegate, this);

			_last_error = CPhidgetMotorControl_set_OnCurrentUpdate_Handler(_motorHandle,
				PhidgetMotor::currentUpdateDelegate, this);
			
		}
	}
}

PhidgetMotor::~PhidgetMotor()
{
}

auto PhidgetMotor::init(int serial_number) -> int
{
	return (_last_error = open(serial_number));
}

auto PhidgetMotor::getMotorCount() -> int
{
	int count = -1;

	_last_error = CPhidgetMotorControl_getMotorCount(_motorHandle, &count);

	return count;
}

auto PhidgetMotor::getVelocity(int index) -> double
{
	double velocity = -1;

	_last_error = CPhidgetMotorControl_getVelocity(_motorHandle, index, &velocity);

	return velocity;
}

auto PhidgetMotor::setVelocity(int index, double velocity) -> void
{
	_last_error = CPhidgetMotorControl_setVelocity(_motorHandle, index, velocity);

	return;
}

auto PhidgetMotor::getAcceleration(int index) -> double
{
	double acceleration = -1;

	_last_error = CPhidgetMotorControl_getAcceleration(_motorHandle, index, &acceleration);

	return acceleration;
}

auto PhidgetMotor::setAcceleration(int index, double acceleration) -> void
{
	_last_error = CPhidgetMotorControl_setAcceleration(_motorHandle, index, acceleration);

	return;
}

auto PhidgetMotor::getAccelerationMax(int index) -> double
{
	double acceleration_max = -1;

	_last_error = CPhidgetMotorControl_getAccelerationMax(_motorHandle, index, &acceleration_max);

	return acceleration_max;
}

auto PhidgetMotor::getAccelerationMin(int index) -> double
{
	double acceleration_min = -1;

	_last_error = CPhidgetMotorControl_getAccelerationMin(_motorHandle, index, &acceleration_min);

	return acceleration_min;
}

auto PhidgetMotor::getCurrent(int index) -> double
{
	double current = -1;

	_last_error = CPhidgetMotorControl_getCurrent(_motorHandle, index, &current);

	return current;
}

auto PhidgetMotor::getInputCount() -> int
{
	int count = -1;

	_last_error = CPhidgetMotorControl_getInputCount(_motorHandle, &count);

	return count;
}

auto PhidgetMotor::getInputState(int index) -> int
{
	int state = -1;

	 _last_error = CPhidgetMotorControl_getInputState(_motorHandle, index,
	 		&state);

	return state;
}

auto PhidgetMotor::getEncoderCount() -> int
{
	int count = -1;

	 _last_error = CPhidgetMotorControl_getEncoderCount(_motorHandle, &count);

	return count;
}

auto PhidgetMotor::getEncoderPosition(int index) -> int
{
	int position = -1;

	 _last_error = CPhidgetMotorControl_getEncoderPosition(_motorHandle, index,  &position);

	return position;
}

auto PhidgetMotor::setEncoderPosition(int index, int position) -> void
{
	 _last_error = CPhidgetMotorControl_setEncoderPosition(_motorHandle, index, position);

	return;
}

auto PhidgetMotor::getBackEMFSensingState(int index) -> int
{
	int position = -1;

	 _last_error = CPhidgetMotorControl_getEncoderPosition(_motorHandle, index,  &position);

	return position;
}

auto PhidgetMotor::setBackEMFSensingState(int index, int bEMFState) -> void
{
	 _last_error = CPhidgetMotorControl_setBackEMFSensingState(_motorHandle, index, bEMFState);

	return;
}

auto PhidgetMotor::getBackEMF(int index) -> double
{
	double emf = -1;

	 _last_error = CPhidgetMotorControl_getBackEMF(_motorHandle, index,  &emf);

	return emf;
}

auto PhidgetMotor::getSupplyVoltage() -> double
{
	double voltage = -1;

	 _last_error = CPhidgetMotorControl_getSupplyVoltage(_motorHandle, &voltage);

	return voltage;
}

auto PhidgetMotor::getBraking(int index) -> double
{
	double braking = -1;

	 _last_error = CPhidgetMotorControl_getBraking(_motorHandle, index, &braking);

	return braking;
}

auto PhidgetMotor::setBraking(int index, double braking) -> void
{
	 _last_error = CPhidgetMotorControl_setBraking(_motorHandle, index, braking);

	return;
}

auto PhidgetMotor::getSensorCount() -> int
{
	int count = -1;

	_last_error = CPhidgetMotorControl_getSensorCount(_motorHandle, &count);

	return count;
}

auto PhidgetMotor::getSensorValue(int index) -> int
{
	int value = -1;

	 _last_error = CPhidgetMotorControl_getSensorValue(_motorHandle, index, &value);

	return value;
}

auto PhidgetMotor::getSensorRawValue(int index) -> int
{
	int value = -1;

	 _last_error = CPhidgetMotorControl_getSensorRawValue(_motorHandle, index, &value);

	return value;
}

auto PhidgetMotor::getRatiometric() -> int
{
	int ratiometric = -1;

	_last_error = CPhidgetMotorControl_getRatiometric(_motorHandle, &ratiometric);

	return ratiometric;
}

auto PhidgetMotor::setRatiometric(int ratiometric) -> int
{
	return (_last_error = CPhidgetMotorControl_setRatiometric(_motorHandle,
			ratiometric));
}

auto PhidgetMotor::getError() -> int
{
	return _last_error;
}

auto PhidgetMotor::attachHandler() -> int
{
	int serialNo, version, numInputs, numOutputs;
	int numSensors, triggerVal, ratiometric, i;
	int numMotors, numEncoders;
	const char *ptr, *name;

	CPhidget_getDeviceName((CPhidgetHandle)_motorHandle, &name);
	CPhidget_getDeviceType((CPhidgetHandle)_motorHandle, &ptr);
	CPhidget_getSerialNumber((CPhidgetHandle)_motorHandle, &serialNo);
	CPhidget_getDeviceVersion((CPhidgetHandle)_motorHandle, &version);

	CPhidgetMotorControl_getMotorCount(_motorHandle, &numMotors);
	CPhidgetMotorControl_getEncoderCount(_motorHandle, &numEncoders);
	CPhidgetMotorControl_getInputCount(_motorHandle, &numInputs);
	CPhidgetMotorControl_getSensorCount(_motorHandle, &numSensors);
	CPhidgetMotorControl_getRatiometric(_motorHandle, &ratiometric);

	printf("%s %d attached!\n", name, serialNo);

	printf("%s", ptr);
	printf("Serial Number: %d\tVersion: %d\n", serialNo, version);
	printf("Num Motors: %d\n", numMotors);
	printf("Num Encoders: %d\n", numEncoders);
	printf("Num Digital Inputs: %d\n", numInputs);
	printf("Num Sensors: %d\n", numSensors);
	printf("Ratiometric: %d\n", ratiometric);

	return 0;
}

auto PhidgetMotor::detachHandler() -> int
{
	int serial_number;
    const char *device_name;

    CPhidget_getDeviceName ((CPhidgetHandle)_motorHandle, &device_name);
    CPhidget_getSerialNumber((CPhidgetHandle)_motorHandle, &serial_number);
    printf("%s Serial number %d detached!\n", device_name, serial_number);
    return 0;
}

auto PhidgetMotor::velocityChangeHandler(int index, double velocity) -> int
{
	return 0;
}

auto PhidgetMotor::currentChangeHandler(int index, double current) -> int
{
	return 0;
}

auto PhidgetMotor::inputChangeHandler(int index, int inputState) -> int
{
	return 0;
}

auto PhidgetMotor::encoderPositionChangeHandler(int index, int time, int positionChange) -> int
{
	return 0;
}

auto PhidgetMotor::encoderPositionUpdateHandler(int index, int inputSpositionChangetate) -> int
{
	return 0;
}

auto PhidgetMotor::backEMFUpdateHandler(int index, double voltage) -> int
{
	return 0;
}

auto PhidgetMotor::sensorUpdateHandler(int index, int sensorValue) -> int
{
	return 0;
}

auto PhidgetMotor::currentUpdateHandler(int index, double current) -> int
{
	return 0;
}

auto PhidgetMotor::attachDelegate(CPhidgetHandle phid, void *userptr) -> int
{
	return ((PhidgetMotor*) userptr)->attachHandler();
}

auto PhidgetMotor::detachDelegate(CPhidgetHandle phid, void *userptr) -> int
{
	return ((PhidgetMotor*) userptr)->detachHandler();
}

auto PhidgetMotor::velocityChangeDelegate(CPhidgetMotorControlHandle handle,
		void *userPtr, int index, double velocity) -> int
{
	return ((PhidgetMotor*) userPtr)->velocityChangeHandler(index, velocity);
}

auto PhidgetMotor::currentChangeDelegate(CPhidgetMotorControlHandle handle,
		void *userPtr, int index, double current) -> int
{
	return ((PhidgetMotor*) userPtr)->currentChangeHandler(index, current);
}

auto PhidgetMotor::inputChangeDelegate(CPhidgetMotorControlHandle handle,
		void *userPtr, int index, int inputState) -> int
{
	return ((PhidgetMotor*) userPtr)->inputChangeHandler(index, inputState);
}

auto PhidgetMotor::encoderPositionChangeDelegate(CPhidgetMotorControlHandle handle,
		void *userPtr, int index, int time, int positionChange) -> int
{
	return ((PhidgetMotor*) userPtr)->encoderPositionChangeHandler(index, time, positionChange);
}

auto PhidgetMotor::encoderPositionUpdateDelegate(CPhidgetMotorControlHandle handle,
		void *userPtr, int index, int positionChange) -> int
{
	return ((PhidgetMotor*) userPtr)->encoderPositionUpdateHandler(index, positionChange);
}

auto PhidgetMotor::backEMFUpdateDelegate(CPhidgetMotorControlHandle handle,
		void *userPtr, int index, double voltage) -> int
{
	return ((PhidgetMotor*) userPtr)->backEMFUpdateHandler(index, voltage);
}

auto PhidgetMotor::sensorUpdateDelegate(CPhidgetMotorControlHandle handle,
		void *userPtr, int index, int sensorValue) -> int
{
	return ((PhidgetMotor*) userPtr)->sensorUpdateHandler(index, sensorValue);
}

auto PhidgetMotor::currentUpdateDelegate(CPhidgetMotorControlHandle handle,
		void *userPtr, int index, double current) -> int
{
	return ((PhidgetMotor*) userPtr)->currentUpdateHandler(index, current);
}


auto PhidgetMotor::update()-> void
{
	printf("PhidgetMotor::update()");
}
