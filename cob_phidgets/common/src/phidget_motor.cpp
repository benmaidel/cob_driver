#include <cob_phidgets/phidget_motor.h>

PhidgetMotor::PhidgetMotor(SensingMode mode)
: Phidget((CPhidgetHandle*) &_motorHandle, mode), _motorHandle(0)
{
	_last_error = CPhidgetInterfaceKit_create(&_iKitHandle);
	//_last_error = CPhidget

	if (!_last_error) {
		CPhidget_set_OnAttach_Handler((CPhidgetHandle) _iKitHandle,
				PhidgetMotor::attachDelegate, this);
		CPhidgetInterfaceKit_set_OnOutputChange_Handler(_iKitHandle,
				PhidgetMotor::outputChangeDelegate, this);

		if(_sensMode == SensingMode::EVENT)
		{
			CPhidgetInterfaceKit_set_OnInputChange_Handler(_iKitHandle,
					PhidgetMotor::inputChangeDelegate, this);
			_last_error = CPhidgetInterfaceKit_set_OnSensorChange_Handler(
					_iKitHandle, PhidgetMotor::sensorChangeDelegate, this);
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

auto PhidgetMotor::getInputCount() -> int
{
	int count = -1;

	_last_error = CPhidgetInterfaceKit_getInputCount(_iKitHandle, &count);

	return count;
}

auto PhidgetMotor::getInputState(int index) -> int
{
	int state = -1;

	 _last_error = CPhidgetInterfaceKit_getInputState(_iKitHandle, index,
	 		&state);

	return state;
}

auto PhidgetMotor::getOutputCount() -> int
{
	int count = -1;

	_last_error = CPhidgetInterfaceKit_getOutputCount(_iKitHandle, &count);

	return count;
}

auto PhidgetMotor::getOutputState(int index) -> int
{
	int state = -1;

	 _last_error = CPhidgetInterfaceKit_getOutputState(_iKitHandle, index,
	 		&state);

	return state;
}

auto PhidgetMotor::setOutputState(int index, int state) -> int
{
	return (_last_error = CPhidgetInterfaceKit_setOutputState(_iKitHandle,
	 		index, state));
}

auto PhidgetMotor::getSensorCount() -> int
{
	int count = -1;

	_last_error = CPhidgetInterfaceKit_getSensorCount(_iKitHandle, &count);

	return count;
}

auto PhidgetMotor::getSensorValue(int index) -> int
{
	int value = -1;

	 _last_error = CPhidgetInterfaceKit_getSensorValue(_iKitHandle, index, &value);

	return value;
}

auto PhidgetMotor::getSensorRawValue(int index) -> int
{
	int value = -1;

	 _last_error = CPhidgetInterfaceKit_getSensorRawValue(_iKitHandle, index, &value);

	return value;
}

auto PhidgetMotor::getSensorChangeTrigger(int index) -> int
{
	int trigger = -1;

	 _last_error = CPhidgetInterfaceKit_getSensorChangeTrigger(_iKitHandle,	index, &trigger);

	return trigger;
}

auto PhidgetMotor::setSensorChangeTrigger(int index, int trigger) -> int
{
	return (_last_error = CPhidgetInterfaceKit_setSensorChangeTrigger(_iKitHandle, index, trigger));
}

auto PhidgetMotor::getRatiometric() -> int
{
	int ratiometric = -1;

	_last_error = CPhidgetInterfaceKit_getRatiometric(_iKitHandle,
			&ratiometric);

	return ratiometric;
}

auto PhidgetMotor::setRatiometric(int ratiometric) -> int
{
	return (_last_error = CPhidgetInterfaceKit_setRatiometric(_iKitHandle,
			ratiometric));
}

auto PhidgetMotor::getDataRate(int index) -> int
{
	int datarate = -1;

	_last_error = CPhidgetInterfaceKit_getDataRate(_iKitHandle, index, &datarate);

	return datarate;
}

auto PhidgetMotor::setDataRate(int index, int datarate) -> int
{
	return (_last_error = CPhidgetInterfaceKit_setDataRate(_iKitHandle,	index, datarate));
}

auto PhidgetMotor::getDataRateMax(int index) -> int
{
	int max = -1;

	_last_error = CPhidgetInterfaceKit_getDataRateMax(_iKitHandle, index, &max);

	return max;
}

auto PhidgetMotor::getDataRateMin(int index) -> int
{
	int min = -1;

	_last_error = CPhidgetInterfaceKit_getDataRateMin(_iKitHandle, index, &min);

	return min;
}

auto PhidgetMotor::getError() -> int
{
	return _last_error;
}

auto PhidgetMotor::attachHandler() -> int
{
	int serialNo, version, numInputs, numOutputs;
	int numSensors, triggerVal, ratiometric, i;
	const char *ptr, *name;

	CPhidget_getDeviceName((CPhidgetHandle)_iKitHandle, &name);
	CPhidget_getDeviceType((CPhidgetHandle)_iKitHandle, &ptr);
	CPhidget_getSerialNumber((CPhidgetHandle)_iKitHandle, &serialNo);
	CPhidget_getDeviceVersion((CPhidgetHandle)_iKitHandle, &version);

	CPhidgetInterfaceKit_getInputCount(_iKitHandle, &numInputs);
	CPhidgetInterfaceKit_getOutputCount(_iKitHandle, &numOutputs);
	CPhidgetInterfaceKit_getSensorCount(_iKitHandle, &numSensors);
	CPhidgetInterfaceKit_getRatiometric(_iKitHandle, &ratiometric);

	printf("%s %d attached!\n", name, serialNo);

	printf("%s", ptr);
	printf("Serial Number: %d\tVersion: %d\n", serialNo, version);
	printf("Num Digital Inputs: %d\tNum Digital Outputs: %d\n", numInputs, numOutputs);
	printf("Num Sensors: %d\n", numSensors);
	printf("Ratiometric: %d\n", ratiometric);

	for(i = 0; i < numSensors; i++)
	{
		CPhidgetInterfaceKit_getSensorChangeTrigger (_iKitHandle, i, &triggerVal);

		printf("Sensor#: %d > Sensitivity Trigger: %d\n", i, triggerVal);
	}

	return 0;
}

auto PhidgetMotor::detachHandler() -> int
{
	int serial_number;
    const char *device_name;

    CPhidget_getDeviceName ((CPhidgetHandle)_iKitHandle, &device_name);
    CPhidget_getSerialNumber((CPhidgetHandle)_iKitHandle, &serial_number);
    printf("%s Serial number %d detached!\n", device_name, serial_number);
    return 0;
}

auto PhidgetMotor::inputChangeHandler(int index, int inputState) -> int
{
	return 0;
}

auto PhidgetMotor::outputChangeHandler(int index, int outputState) -> int
{
	return 0;
}

auto PhidgetMotor::sensorChangeHandler(int index, int sensorValue) -> int
{
	return 0;
}

auto PhidgetMotor::attachDelegate(CPhidgetHandle phid, void *userptr) -> int
{
	return ((PhidgetMotor*) userptr)->attachHandler();
}

auto PhidgetMotor::inputChangeDelegate(CPhidgetInterfaceKitHandle phid,
		void *userPtr, int index, int inputState) -> int
{
	return ((PhidgetMotor*) userPtr)->inputChangeHandler(index, inputState);
}

auto PhidgetMotor::outputChangeDelegate(CPhidgetInterfaceKitHandle phid,
		void *userPtr, int index, int outputState) -> int
{
	return ((PhidgetMotor*) userPtr)->outputChangeHandler(index, outputState);
}

auto PhidgetMotor::sensorChangeDelegate(CPhidgetInterfaceKitHandle phid,
		void *userPtr, int index, int sensorValue) -> int
{
	return ((PhidgetMotor*) userPtr)->sensorChangeHandler(index, sensorValue);
}

auto PhidgetMotor::update()-> void
{
	printf("PhidgetMotor::update()");
}
