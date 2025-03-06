#ifndef DYNAMIC_OBSTACLES_H
#define DYNAMIC_OBSTACLES_H

#include <string>
#include <vector>
#include <ros/ros.h>
#include <mutex>

namespace dsvplanner_ns {

// 动态障碍物结构体
struct DynamicObstacle {
    std::string name;        // 障碍物名称
    double x, y, z;          // 位置
    double vx, vy, vz;       // 速度
    double distance;         // 与机器人的距离
};

// 动态障碍物管理类
class DynamicObstacleManager {
public:
    static DynamicObstacleManager& getInstance() {
        static DynamicObstacleManager instance;
        return instance;
    }

    // 更新障碍物列表
    void updateObstacles(const std::vector<DynamicObstacle>& obstacles) {
        std::lock_guard<std::mutex> lock(mutex_);
        obstacles_ = obstacles;
    }

    // 获取障碍物列表
    std::vector<DynamicObstacle> getObstacles() {
        std::lock_guard<std::mutex> lock(mutex_);
        return obstacles_;
    }

private:
    DynamicObstacleManager() {}  // 私有构造函数
    std::vector<DynamicObstacle> obstacles_;
    std::mutex mutex_;  // 保护并发访问
};

} // namespace dsvplanner_ns

#endif // DYNAMIC_OBSTACLES_H