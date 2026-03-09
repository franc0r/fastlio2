#pragma once
#include "imu_processor.h"
#include "lidar_processor.h"
#include "velocity_processor.h"

enum BuilderStatus
{
    IMU_INIT,
    MAP_INIT,
    MAPPING
};

class MapBuilder
{
public:
    MapBuilder(Config &config, std::shared_ptr<IESKF> kf);

    void process(SyncPackage &package);
    BuilderStatus status() { return m_status; }    
    std::shared_ptr<LidarProcessor> lidar_processor(){return m_lidar_processor;}
    std::shared_ptr<VelocityProcessor> velocity_processor(){return m_velocity_processor;}

private:
    Config m_config;
    BuilderStatus m_status;
    std::shared_ptr<IESKF> m_kf;
    std::shared_ptr<IMUProcessor> m_imu_processor;
    std::shared_ptr<LidarProcessor> m_lidar_processor;
    std::shared_ptr<VelocityProcessor> m_velocity_processor;
};
