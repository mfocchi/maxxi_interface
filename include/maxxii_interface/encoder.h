#ifndef ENCODER_H
#define ENCODER_H

#include <vector>
#include <iostream>
#include <memory>
#include <string>
#include <cmath>
#include "driver_interface.h"

class Encoder
{
private:
    double pulse_per_revolution;
    double reference_pulse_count;
    int id;
    std::shared_ptr<DriverInterface> Device;
public:
    Encoder(int id, double ppr, std::shared_ptr<DriverInterface> Device);
    
    void setReferencePulseCount(double pulse_count);
    
    double getSpeedRPM() const;
    double getPulseCount() const;
    double getRevolutions() const;
    double getRadiants() const;
    double getDegrees() const;
    double getSpeedRads() const;

};

#endif