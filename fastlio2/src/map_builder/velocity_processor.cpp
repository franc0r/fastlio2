#include "velocity_processor.h"

VelocityProcessor::VelocityProcessor(Config &config, std::shared_ptr<IESKF> kf) 
    : m_config(config), m_kf(kf)
{
    // Initialisiere Kovaranzmatrix aus Config
    m_velocity_cov = M3D::Zero();
    m_velocity_cov(0, 0) = config.velocity_cov_x;
    m_velocity_cov(1, 1) = config.velocity_cov_y;
    m_velocity_cov(2, 2) = config.velocity_cov_z;
}

void VelocityProcessor::addMeasurement(const V3D &velocity, double timestamp)
{
    m_velocity_buffer.emplace_back(velocity, timestamp, m_velocity_cov);
}

void VelocityProcessor::processMeasurements()
{
    if (m_velocity_buffer.empty())
        return;

    for (const auto &measurement : m_velocity_buffer)
    {
        // Update IESKF mit Geschwindigkeitsmessung
        m_kf->updateVelocity(measurement.velocity, measurement.cov);
    }

    m_velocity_buffer.clear();
}
