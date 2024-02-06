#ifndef MOTOR_H
#define MOTOR_H

#include <vector>
#include <iostream>
#include <memory>
#include <string>
#include <cmath>
#include "driver_interface.h"

#define RADPS_TO_RPM (30.0 / M_PI)
#define DEGPS_TO_RPM (1.0/5.0)

typedef enum Motor_ctrl_mode {VELOCITY, TORQUE} Motor_ctrl_mode;

class Motor
{
private:
    double torque_constant;
    double speed_ctrl;
    double sprocket_radius;
    int id;
    Motor_ctrl_mode mode;
    std::shared_ptr<DriverInterface> Device;
public:
    Motor(int id, Motor_ctrl_mode mode, double torque_constant, std::shared_ptr<DriverInterface> Device);
    Motor(int id, std::shared_ptr<DriverInterface> Device);
    
    void moveRPM(double rpm);
    void moveDEGPS(double deg_per_sec);
    void moveRADPS(double rad_per_sec);
    void moveMPS(double m_per_sec);
    void applyTorque(double tau);
    void applyCurrent(double amp);

    double getVoltage() const;
    double getCurrent() const;
    double getTemperature() const;

    bool isVelocityMode() {return mode == VELOCITY;}
    bool isTorqueMode() {return mode == TORQUE;}
};

#endif