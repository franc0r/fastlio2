#pragma once
#include "ieskf.h"
#include "commons.h"

struct VelocityMeasurement
{
    V3D velocity;
    double timestamp;
    M3D cov; // Kovarianzmatrix für die Geschwindigkeitsmessung
    
    VelocityMeasurement() = default;
    VelocityMeasurement(const V3D &v, double t, const M3D &c) : velocity(v), timestamp(t), cov(c) {}
};

class VelocityProcessor
{
public:
    VelocityProcessor(Config &config, std::shared_ptr<IESKF> kf);

    void addMeasurement(const V3D &velocity, double timestamp);

    void processMeasurements();

    bool hasMeasurements() const { return !m_velocity_buffer.empty(); }

private:
    Config m_config;
    std::shared_ptr<IESKF> m_kf;
    std::vector<VelocityMeasurement> m_velocity_buffer;
    M3D m_velocity_cov;
};
