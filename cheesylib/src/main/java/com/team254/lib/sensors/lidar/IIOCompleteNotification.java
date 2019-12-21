package com.team254.lib.sensors.lidar;

public interface IIOCompleteNotification {
    void setLidarData(VL53L0XProtocol.LidarData data, double sensor_timestamp);
}
