#ifndef _PHIDGET_MOTOR_H_
#define _PHIDGET_MOTOR_H_

#include <cob_phidgets/phidget.h>

class PhidgetMotor: public Phidget
{
public:
	PhidgetMotor(SensingMode mode);
	~PhidgetMotor();

	auto init(int serial_number) -> int;

	auto getMotorCount() -> int;

	auto getVelocity(int index) -> double;
	auto setVelocity(int index, double velocity) -> void;

	auto getAcceleration(int index) -> double;
	auto setAcceleration(int index, double acceleration) -> void;
	auto getAccelerationMin(int index) -> double;
	auto getAccelerationMax(itn index) -> double;

	auto getCurrent(int index) -> double;

	auto getInputCount() -> int;
	auto getInputState(int index) -> int;

	auto getEncoderCount() -> int;
	auto getEncoderPosition(int index) -> int;
	auto setEncoderPosition(int index, int position) ->void;

	auto getBackEMFSensingState(int index) -> int;
	auto setBackEMFSensingSTate(int index, int bEMFState) -> void;
	auto getBackEMF(int index) -> double;

	auto getSupplyVoltage() -> double;

	auto getBraking(int index) -> double;
	auto setBraking(int index, double braking) -> void;

	auto getSensorCount() -> int;
	auto getSensorValue(int index) -> int;
	auto getSensorRawValue(int index) -> int;

	auto getRatiometric() -> int;
	auto setRatiometric(int ratiometric) -> int;

	auto getError() -> int;

	virtual auto update() -> void;

protected:
	CPhidgetMotorControlHandle _motorHandle;

	virtual int attachHandler();
	virtual int detachHandler();

	virtual int velocityChangeHandler(int index, double velocity);
	virtual int currentChangeHandler(int index, double current);
	virtual int inputChangeHandler(int index, int inputState);
	virtual int encoderPositionChangeHandler(int index, int time, int positionChange);
	virtual int encoderPositionUpdateHandler(int index, int positionChange);

	virtual int backEMFUpdateHandler(int index, double voltage);

	virtual int sensorUpdateHandler(int index, int sensorValue);
	virtual int currentUpdateHandler(int index, double current);

private:
	static auto attachDelegate(CPhidgetHandle phid, void *userptr) -> int;

	static auto velocityChangeDelegate(CPhidgetMotorControlHandle phid,
			void *userPtr, int index, double velocity) -> int;

	static auto currentChangeDelegate(CPhidgetMotorControlHandle phid,
			void *userPtr, int index, double current) -> int;

	static auto inputChangeDelegate(CPhidgetMotorControlHandle phid,
			void *userPtr, int index, int inputState) -> int;

	static auto encoderPositionChangeDelegate(CPhidgetMotorControlHandle phid,
			void *userPtr, int index, int time, int positionChange) -> int;

	static auto encoderPositionUpdateDelegate(CPhidgetMotorControlHandle phid,
			void *userPtr, int index, int positionChange) -> int;

	static auto backEMFUpdateDelegate(CPhidgetMotorControlHandle phid,
			void *userPtr, int index, int positionChange) -> int;

	static auto sensorUpdateDelegate(CPhidgetMotorControlHandle phid,
			void *userPtr, int index, int sensorValue) -> int;

	static auto currentUpdateDelegate(CPhidgetMotorControlHandle phid,
			void *userPtr, int index, double current) -> int;
};
#endif //_PHIDGET_MOTOR_H_
