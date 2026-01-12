/**
 * @file mid360_ode_multiray_shape.h
 * @brief ODE 物理引擎多射线形状类
 *
 * 继承自 Gazebo MultiRayShape，实现 Mid-360 激光雷达的射线碰撞检测
 * 使用 ODE (Open Dynamics Engine) 进行高效的射线-物体碰撞计算
 */

#ifndef MID360_SIMULATION_ODE_MULTIRAY_SHAPE_H
#define MID360_SIMULATION_ODE_MULTIRAY_SHAPE_H

#include <gazebo/physics/MultiRayShape.hh>
#include <gazebo/util/system.hh>
#include <gazebo/ode/common.h>
#include <ignition/math.hh>

namespace gazebo
{
namespace physics
{

/**
 * @class Mid360OdeMultiRayShape
 * @brief Mid-360 激光雷达 ODE 多射线形状
 *
 * 管理激光雷达的所有射线，处理与场景中物体的碰撞检测
 */
class GZ_PHYSICS_VISIBLE Mid360OdeMultiRayShape : public MultiRayShape
{
public:
    /**
     * @brief 构造函数
     * @param _parent 父碰撞体指针
     */
    explicit Mid360OdeMultiRayShape(CollisionPtr _parent);

    /**
     * @brief 析构函数
     */
    virtual ~Mid360OdeMultiRayShape();

    /**
     * @brief 更新所有射线的碰撞检测
     */
    virtual void UpdateRays();

    /**
     * @brief 初始化射线形状
     */
    virtual void Init();

    /**
     * @brief 获取射线形状列表的引用
     * @return 射线形状指针向量的引用
     */
    std::vector<RayShapePtr>& RayShapes() { return rays; }

    /**
     * @brief 添加一条射线
     * @param _start 射线起点
     * @param _end 射线终点
     */
    void AddRay(const ignition::math::Vector3d& _start,
                const ignition::math::Vector3d& _end);

private:
    /**
     * @brief 射线碰撞回调函数
     * @param _data 用户数据指针
     * @param _o1 第一个几何体 ID
     * @param _o2 第二个几何体 ID
     */
    static void UpdateCallback(void* _data, dGeomID _o1, dGeomID _o2);

    /// @brief 包含射线空间的父空间 ID
    dSpaceID superSpaceId;

    /// @brief 射线碰撞检测空间 ID
    dSpaceID raySpaceId;

    /// @brief 射线形状列表
    std::vector<RayShapePtr> livoxRays;
};

} // namespace physics
} // namespace gazebo

#endif // MID360_SIMULATION_ODE_MULTIRAY_SHAPE_H
