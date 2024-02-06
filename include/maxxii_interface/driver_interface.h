#ifndef DRIVER_INTERFACE_H
#define DRIVER_INTERFACE_H

#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <cmath>
// #include "../include/maxxii_interface/RoboteqDevice.h"
#include "RoboteqDevice.h"
#include "error_codes.h"

#define LEFT_ID 1
#define RIGHT_ID 2
#define CTRL_MOTOR_MAX_VALUE 1000

typedef enum Position {LEFT, RIGHT} Position;


double mapRange(double val1, double max1, double max2);
double saturate(double val, double max_val);

class Couple
{
private:
    std::vector<double> value;
public:
    Couple(){set(0.0, 0.0);}
    Couple(double left_val, double right_val) 
    {
        set(left_val, right_val);
    }
    Couple(const Couple& other) {set(other.getLeft(), other.getRight());}
    double getLeft() const {return get(LEFT_ID);}
    double getRight() const {return get(RIGHT_ID);}
    double get(int index) const {
        if(index == LEFT_ID)
            return value.at(0);
        else if(index == RIGHT_ID)
            return value.at(1);
        else
        {
            throw WORNG_INDEXING;
            return 0.0;
        }
    }
    void setLeft(double val) {value.at(LEFT_ID) = val;}
    void setRight(double val) {value.at(RIGHT_ID) = val;}
    void set(double left_val, double right_val) {value = {left_val, right_val};}
};

class DriverInterface
{
public:
    virtual double getEncoderCount(int id) const = 0;
    virtual double getEncoderSpeed(int id) const = 0;
    virtual double getFeedback(int id) const = 0;
    virtual double getMotorCurrent(int id) const = 0;
    virtual double getMotorVoltage(int id) const = 0;
    virtual void getInOutA(std::string* aio) const = 0;
    virtual void getInOutD(std::string* dio) const = 0;
    virtual double getTemperature(int id) const = 0;

    virtual void measureEncodersCount() = 0;
    virtual void measureEncodersSpeedRPM() = 0;
    virtual void measureFeedback() = 0;
    virtual void measureMotorsCurrent() = 0;
    virtual void measureMotorsVoltage() = 0;
    virtual void measureTemperature() = 0;

    virtual void setMotorTorque(int id, double tau) = 0;
    virtual void setMotorSpeed(int id, double velocity) = 0;
    virtual void setMotorCurrent(int id, double current) = 0;
    
    virtual void resetEncoder(int id) = 0;
    virtual void printFirmware() = 0;
     
    virtual void commandInputOutput(int out_0, int out_1) = 0;
    
};


class RoboteqDriverInterface : public DriverInterface 
{
private:
    Couple pulse_count;
    Couple encoder_speed;
    Couple feedback;
    Couple motorsAmps;
    Couple motorsVolt;
    Couple temperature;

    double max_velocity;
    double max_torque;
    double max_current;

    RoboteqDevice device;

    double extractMeasurement(const std::string& reading_str, Position position);
    
    void readEncodersCount(std::string* count); 
    void readEncodersSpeed(std::string* speed); 
    void readFeedback(std::string* fbk); 
    void readMotorsCurrent(std::string* current); 
    void readMotorsVoltage(std::string* voltage); 
    void readFirmware(std::string* firmware);
    void readTemperature(std::string* temp); 

    void sendMotorCmd(int id, double cmd_value);
    void connectToDevice(std::string port);

public:
    RoboteqDriverInterface(double max_vel, double max_amp, double max_tau, std::string port);
    
    double getEncoderCount(int id) const override;
    double getEncoderSpeed(int id) const override;
    double getFeedback(int id) const override;
    double getMotorCurrent(int id) const override;
    double getMotorVoltage(int id) const override;
    void getInOutA(std::string* aio) const override {*aio = "";/*TODO*/}
    void getInOutD(std::string* dio) const override {*dio = "";/*TODO*/}
    double getTemperature(int id) const override;


    void measureEncodersCount() override;
    void measureEncodersSpeedRPM() override;
    void measureFeedback() override;
    void measureMotorsCurrent() override;
    void measureMotorsVoltage() override;
    void measureTemperature() override;

    void setMotorTorque(int id, double tau) override;
    void setMotorSpeed(int id, double velocity) override;
    void setMotorCurrent(int id, double current) override;
    
    void resetEncoder(int id) override;
    void printFirmware() override;

    void commandInputOutput(int out_0, int out_1) override;
};



#endif

