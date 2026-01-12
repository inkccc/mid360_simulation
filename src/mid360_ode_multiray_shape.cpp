/**
 * @file mid360_ode_multiray_shape.cpp
 * @brief ODE 多射线形状实现
 *
 * 实现 Mid-360 激光雷达的射线碰撞检测功能
 */

#include <gazebo/common/Assert.hh>
#include <gazebo/common/Exception.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/ode/ODETypes.hh>
#include <gazebo/physics/ode/ODELink.hh>
#include <gazebo/physics/ode/ODECollision.hh>
#include <gazebo/physics/ode/ODEPhysics.hh>
#include <gazebo/physics/ode/ODERayShape.hh>
#include <gazebo/physics/ode/ODEMultiRayShape.hh>

#include "mid360_simulation/mid360_ode_multiray_shape.h"

using namespace gazebo;
using namespace physics;

//==============================================================================
// 构造函数
//==============================================================================
Mid360OdeMultiRayShape::Mid360OdeMultiRayShape(CollisionPtr _parent)
    : MultiRayShape(_parent)
{
    this->SetName("Mid360 ODE Multiray Shape");

    // 创建包含射线空间的父空间
    this->superSpaceId = dSimpleSpaceCreate(0);

    // 创建射线碰撞检测空间
    this->raySpaceId = dSimpleSpaceCreate(this->superSpaceId);

    // 设置碰撞位掩码 - 传感器只与非传感器物体碰撞
    dGeomSetCategoryBits((dGeomID)this->raySpaceId, GZ_SENSOR_COLLIDE);
    dGeomSetCollideBits((dGeomID)this->raySpaceId, ~GZ_SENSOR_COLLIDE);

    // 设置父链接的空间 ID
    ODELinkPtr pLink =
        boost::static_pointer_cast<ODELink>(this->collisionParent->GetLink());
    pLink->SetSpaceId(this->raySpaceId);
    boost::static_pointer_cast<ODECollision>(this->collisionParent)->SetSpaceId(this->raySpaceId);
}

//==============================================================================
// 析构函数
//==============================================================================
Mid360OdeMultiRayShape::~Mid360OdeMultiRayShape()
{
    // 清理射线空间
    dSpaceSetCleanup(this->raySpaceId, 0);
    dSpaceDestroy(this->raySpaceId);

    // 清理父空间
    dSpaceSetCleanup(this->superSpaceId, 0);
    dSpaceDestroy(this->superSpaceId);
}

//==============================================================================
// 更新射线碰撞检测
//==============================================================================
void Mid360OdeMultiRayShape::UpdateRays()
{
    ODEPhysicsPtr ode = boost::dynamic_pointer_cast<ODEPhysics>(
        this->GetWorld()->Physics());

    if (ode == NULL)
        gzthrow("物理引擎无效，必须使用 ODE 引擎");

    // 锁定物理引擎进行碰撞检测
    {
        boost::recursive_mutex::scoped_lock lock(*ode->GetPhysicsUpdateMutex());

        // 执行空间碰撞检测
        dSpaceCollide2((dGeomID)(this->superSpaceId),
                       (dGeomID)(ode->GetSpaceId()),
                       this, &UpdateCallback);
    }
}

//==============================================================================
// 碰撞检测回调函数
//==============================================================================
void Mid360OdeMultiRayShape::UpdateCallback(void* _data, dGeomID _o1, dGeomID _o2)
{
    dContactGeom contact;
    Mid360OdeMultiRayShape* self = static_cast<Mid360OdeMultiRayShape*>(_data);

    // 处理空间碰撞
    if (dGeomIsSpace(_o1) || dGeomIsSpace(_o2))
    {
        if (dGeomGetSpace(_o1) == self->superSpaceId ||
            dGeomGetSpace(_o2) == self->superSpaceId)
            dSpaceCollide2(_o1, _o2, self, &UpdateCallback);

        if (dGeomGetSpace(_o1) == self->raySpaceId ||
            dGeomGetSpace(_o2) == self->raySpaceId)
            dSpaceCollide2(_o1, _o2, self, &UpdateCallback);
    }
    else
    {
        ODECollision* collision1 = NULL;
        ODECollision* collision2 = NULL;

        // 获取碰撞体指针
        if (dGeomGetClass(_o1) == dGeomTransformClass)
        {
            collision1 = static_cast<ODECollision*>(
                dGeomGetData(dGeomTransformGetGeom(_o1)));
        }
        else
        {
            collision1 = static_cast<ODECollision*>(dGeomGetData(_o1));
        }

        if (dGeomGetClass(_o2) == dGeomTransformClass)
        {
            collision2 = static_cast<ODECollision*>(
                dGeomGetData(dGeomTransformGetGeom(_o2)));
        }
        else
        {
            collision2 = static_cast<ODECollision*>(dGeomGetData(_o2));
        }

        GZ_ASSERT(collision1, "collision1 为空");
        GZ_ASSERT(collision2, "collision2 为空");

        ODECollision* rayCollision = NULL;
        ODECollision* hitCollision = NULL;

        // 确定哪个是射线碰撞体
        if (dGeomGetClass(_o1) == dRayClass)
        {
            rayCollision = collision1;
            hitCollision = collision2;
            dGeomRaySetParams(_o1, 0, 0);
            dGeomRaySetClosestHit(_o1, 1);
        }
        else if (dGeomGetClass(_o2) == dRayClass)
        {
            rayCollision = collision2;
            hitCollision = collision1;
            dGeomRaySetParams(_o2, 0, 0);
            dGeomRaySetClosestHit(_o2, 1);
        }

        // 检测射线与物体的交点
        if (rayCollision && hitCollision)
        {
            int n = dCollide(_o1, _o2, 1, &contact, sizeof(contact));

            if (n > 0)
            {
                RayShapePtr shape = boost::static_pointer_cast<RayShape>(
                    rayCollision->GetShape());

                // 更新射线长度为碰撞距离
                if (contact.depth < shape->GetLength())
                {
                    shape->SetLength(contact.depth);
                    shape->SetRetro(hitCollision->GetLaserRetro());
                }
            }
        }
    }
}

//==============================================================================
// 添加射线
//==============================================================================
void Mid360OdeMultiRayShape::AddRay(const ignition::math::Vector3d& _start,
                                    const ignition::math::Vector3d& _end)
{
    MultiRayShape::AddRay(_start, _end);

    // 创建 ODE 碰撞体
    ODECollisionPtr odeCollision(new ODECollision(
        this->collisionParent->GetLink()));
    odeCollision->SetName("mid360_ray_collision");
    odeCollision->SetSpaceId(this->raySpaceId);

    // 创建射线形状
    ODERayShapePtr ray(new ODERayShape(odeCollision));
    odeCollision->SetShape(ray);

    ray->SetPoints(_start, _end);
    this->rays.push_back(ray);
}

//==============================================================================
// 初始化
//==============================================================================
void Mid360OdeMultiRayShape::Init()
{
    // 从 SDF 读取配置参数
    this->rayElem = this->sdf->GetElement("ray");
    this->scanElem = this->rayElem->GetElement("scan");
    this->horzElem = this->scanElem->GetElement("horizontal");
    this->rangeElem = this->rayElem->GetElement("range");

    if (this->scanElem->HasElement("vertical"))
    {
        this->vertElem = this->scanElem->GetElement("vertical");
    }

    // 射线的实际创建在 Mid360PointsPlugin::Load() 中完成
    // 这里只进行基本初始化
}
