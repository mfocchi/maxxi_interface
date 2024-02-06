#include "encoder.h"


Encoder::Encoder(int id, double ppr, std::shared_ptr<DriverInterface> Device)
{
    this->pulse_per_revolution = ppr;
    this->id = id;
    this->Device = Device;
    
    reference_pulse_count = 0.0;
}

void Encoder::setReferencePulseCount(double pulse_count)
{
    reference_pulse_count = pulse_count;
}

double Encoder::getPulseCount() const
{
    Device->measureEncodersCount();
    double pulse_count = Device->getEncoderCount(id);
    return pulse_count - reference_pulse_count;
}

double Encoder::getRevolutions() const
{
    return getPulseCount() / pulse_per_revolution;
}

double Encoder::getRadiants() const
{
    return getRevolutions() * 2 * M_PI;
}

double Encoder::getDegrees() const
{
    return getRevolutions() * 360.0;
}

double Encoder::getSpeedRPM() const
{
    Device->measureEncodersSpeedRPM();
    double speed = Device->getEncoderSpeed(id);
    return speed;
}

double Encoder::getSpeedRads() const
{
    Device->measureEncodersSpeedRPM();
    double speed = Device->getEncoderSpeed(id);
    return speed*2 * M_PI/60.0;
}