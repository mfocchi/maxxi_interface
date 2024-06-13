#include "driver_interface.h"

RoboteqDriverInterface::RoboteqDriverInterface(double max_vel, double max_amp, double max_tau, std::string port)
{
    device = RoboteqDevice();
    connectToDevice(port);
    resetEncoder(0);
    resetEncoder(1);
    this->max_velocity = max_vel;
    this->max_current = max_amp;
    this->max_torque = max_tau;
    device.setMaxRPM(max_vel);
}

void RoboteqDriverInterface::connectToDevice(std::string port)
{
    int status = device.Connect(port);
    if (status != RQ_SUCCESS)
    {    
        throw FAILED_CONNECTION;
    }
}

double RoboteqDriverInterface::getEncoderCount(int id) const
{
    return pulse_count.get(id);
}
double RoboteqDriverInterface::getEncoderSpeed(int id) const
{
    return encoder_speed.get(id);
}
double RoboteqDriverInterface::getFeedback(int id) const
{
    return feedback.get(id);
}
double RoboteqDriverInterface::getMotorCurrent(int id) const
{
    return motorsAmps.get(id);
}
double RoboteqDriverInterface::getMotorVoltage(int id) const
{
    return motorsVolt.get(id);
}
double RoboteqDriverInterface::getTemperature(int id) const
{
    return temperature.get(id);
}

void RoboteqDriverInterface::measureEncodersSpeedRPM()
{
    std::string speed;
    readEncodersSpeed(&speed);
    double speed_left = extractMeasurement(speed, LEFT);
    double speed_right = extractMeasurement(speed, RIGHT);
    encoder_speed.set(speed_left, speed_right);
}

void RoboteqDriverInterface::measureEncodersCount()
{
    std::string count;
    readEncodersCount(&count);
    double pulses_left = extractMeasurement(count, LEFT);
    double pulses_right = extractMeasurement(count, RIGHT);
    pulse_count.set(pulses_left, pulses_right);
}

void RoboteqDriverInterface::measureFeedback()
{
    std::string fbk;
    readFeedback(&fbk);
    double fbk_left = extractMeasurement(fbk, LEFT);
    double fbk_right = extractMeasurement(fbk, RIGHT);
    feedback.set(fbk_left, fbk_right);
}

void RoboteqDriverInterface::measureMotorsCurrent()
{
    std::string currents;
    readMotorsCurrent(&currents);
    double ampere_left = extractMeasurement(currents, LEFT);
    double ampere_right = extractMeasurement(currents, RIGHT);
    motorsAmps.set(ampere_left, ampere_right);
}

void RoboteqDriverInterface::measureMotorsVoltage()
{
    std::string voltage;
    readMotorsCurrent(&voltage);
    double volt_left = extractMeasurement(voltage, LEFT);
    double volt_right = extractMeasurement(voltage, RIGHT);
    motorsVolt.set(volt_left, volt_right);
}

void RoboteqDriverInterface::measureTemperature()
{
    std::string temp;
    readTemperature(&temp);
    double temp_left = extractMeasurement(temp, LEFT);
    double temp_right = extractMeasurement(temp, RIGHT);
    temperature.set(temp_left, temp_right);
}

double RoboteqDriverInterface::extractMeasurement(const std::string& reading_str, Position position) 
{ 
    size_t colonPos = reading_str.find(':');
    std::string reading;
    if (position == LEFT) 
    {
        reading = reading_str.substr(0, colonPos);
    }
    else if (position == RIGHT) 
    {
        reading = reading_str.substr(colonPos + 1);
    }
    return std::stoi(reading);
}

void RoboteqDriverInterface::readEncodersCount(std::string* count)
{
    int status = device.GetValue(_C, 0, *count);
    if (status != RQ_SUCCESS)
    {    
        throw FAILED_READ_ENCODER_COUNT;
    }
}

void RoboteqDriverInterface::readEncodersSpeed(std::string* speed)
{
    int status = device.GetValue(_S, 0, *speed);
    if (status != RQ_SUCCESS)
    {    
        throw FAILED_READ_ENCODER_SPEED;
    }
}

void RoboteqDriverInterface::readFeedback(std::string* fbk)
{
    int status = device.GetValue(_F, 0, *fbk);
    if (status != RQ_SUCCESS)
    {    
        throw FAILED_READ_FEEDBACK;
    }
}

void RoboteqDriverInterface::readMotorsCurrent(std::string* current)
{
    int status = device.GetValue(_A, 0, *current);
    if (status != RQ_SUCCESS)
    {    
        throw FAILED_READ_MOTOR_CURRENT;
    }
}

void RoboteqDriverInterface::readMotorsVoltage(std::string* voltage)
{
    int status = device.GetValue(_V, 0, *voltage);
    if (status != RQ_SUCCESS)
    {    
        throw FAILED_READ_MOTOR_VOLTAGE;
    }
}

void RoboteqDriverInterface::readFirmware(std::string* firmware)
{
    int status = device.GetValue(_FID, 0, *firmware);
    if (status != RQ_SUCCESS)
    {    
        throw FAILED_READ_FIRMWARE;
    }
}

void RoboteqDriverInterface::readTemperature(std::string* temp)
{
    int status = device.GetValue(_T, 0, *temp);
    if (status != RQ_SUCCESS)
    {    
        throw FAILED_READ_TEMPERATURE;
    }
}


void RoboteqDriverInterface::resetEncoder(int id)
{
    int status = device.SetCommand(_C, id, 0);
    if (status != RQ_SUCCESS)
    {    
        throw FAILED_ENCODER_RESET;
    }
}

void RoboteqDriverInterface::setMotorTorque(int id, double tau)
{
    double tau_norm = mapRange(tau, max_torque, CTRL_MOTOR_MAX_VALUE);
    tau_norm = saturate(tau_norm, CTRL_MOTOR_MAX_VALUE);
    sendMotorCmd(id, tau_norm);
}

void RoboteqDriverInterface::setMotorCurrent(int id, double current)
{
    double current_norm = mapRange(current, max_current, CTRL_MOTOR_MAX_VALUE);
    current_norm = saturate(current_norm, CTRL_MOTOR_MAX_VALUE);
    sendMotorCmd(id, current_norm);
}

void RoboteqDriverInterface::setMotorSpeed(int id, double velocity)
{
    double velocity_norm = mapRange(velocity, max_velocity, CTRL_MOTOR_MAX_VALUE);
    velocity_norm = saturate(velocity_norm, CTRL_MOTOR_MAX_VALUE);  
    sendMotorCmd(id, velocity_norm);
}

void RoboteqDriverInterface::sendMotorCmd(int id, double cmd_value)
{
    int status = device.SetCommand(_GO, id, cmd_value);
    if (status != RQ_SUCCESS)
    {   
        throw FAILED_SET_MOTOR_CMD;
    }
}

void RoboteqDriverInterface::commandInputOutput(int out_0, int out_1) 
{
    // 1  Setting D1 OUT
    // 0  Resetting D1 OUT 
    int status;
    if (out_0 == 1) {
        status = device.SetCommand(_D1, 1);
    } else if (out_0 == 0) {
        status = device.SetCommand(_D0, 1);
    }
    if (status != RQ_SUCCESS)
    {    
        throw FAILED_D1_OUT; 
    }

    // 1  Setting D1 OUT
    // 0  Resetting D1 OUT 
    if (out_1 == 1) {
        status = device.SetCommand(_D1, 2);
    } else if (out_1 == 0) {
        status = device.SetCommand(_D0, 2);
    }
    if (status != RQ_SUCCESS)
    {    
        throw FAILED_D2_OUT;
    }
}

void RoboteqDriverInterface::printFirmware()
{
    std::string firmware;
    readFirmware(&firmware);
    std::cout << "Firmware:" << std::endl;
    std::cout << firmware << std::endl;
}


double mapRange(double val1, double max1, double max2)
{
    double val2 = 0.0;
    if(max1 != 0.0)
        val2 = val1 * max2 / max1;
    return val2;
}

double saturate(double val, double max_val)
{
    if(val > max_val)
        return max_val;
    else if(val < -max_val)
        return -max_val;
    else
        return val;
}