#include "motor.h"

Motor::Motor(int id, std::shared_ptr<DriverInterface> Device)
{
    this->id = id;
    this->Device = Device;
    this->torque_constant = 0.0;
    this->mode = VELOCITY;
}

Motor::Motor(int id, Motor_ctrl_mode mode, double torque_constant, std::shared_ptr<DriverInterface> Device)
{
    this->id = id;
    this->Device = Device;
    this->torque_constant = torque_constant;
    this->mode = mode;
}



void Motor::moveMPS(double m_per_sec)
{
    double rad_per_sec = m_per_sec / sprocket_radius;
    moveRADPS(rad_per_sec);
}
void Motor::moveRADPS(double rad_per_sec)
{
    double rpm = RADPS_TO_RPM * rad_per_sec;
    moveRPM(rpm);
}
void Motor::moveDEGPS(double deg_per_sec)
{
    double rpm = DEGPS_TO_RPM * deg_per_sec;
    moveRPM(rpm);
}
void Motor::moveRPM(double rpm)
{
    if(isVelocityMode())
    {
        Device->setMotorSpeed(id, rpm);
    }
    else
    {
        throw NEED_VELOCITY_MODE;
    }
}

void Motor::applyTorque(double tau)
{   
    double amp = tau * torque_constant;
    applyCurrent(amp);
}
void Motor::applyCurrent(double amp)
{
    if(isTorqueMode())
    {
        Device->setMotorSpeed(id, amp);
    }
    else
    {
        throw NEED_TORQUE_MODE;
    }
}

double Motor::getVoltage() const
{
    Device->measureMotorsVoltage();
    return Device->getMotorVoltage(id);
}
double Motor::getCurrent() const
{
    Device->measureMotorsCurrent();
    return Device->getMotorCurrent(id);
}
double Motor::getTemperature() const
{
    Device->measureTemperature();
    return Device->getTemperature(id);
}