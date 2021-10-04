#ifndef COSTFUNCTIONFACTORY_H
#define COSTFUNCTIONFACTORY_H

#include <boost/shared_ptr.hpp>
#include <opencv2/core/core.hpp>

#include "camera_models/Camera.h"

namespace ceres
{
class CostFunction;
}

namespace camera_model
{

enum
{
    CAMERA_INTRINSICS         = 1 << 0,
    CAMERA_POSE               = 1 << 1,
    POINT_3D                  = 1 << 2,
    ODOMETRY_INTRINSICS       = 1 << 3,
    ODOMETRY_3D_POSE          = 1 << 4,
    ODOMETRY_6D_POSE          = 1 << 5,
    CAMERA_ODOMETRY_TRANSFORM = 1 << 6
};

class CostFunctionFactory
{
    public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    CostFunctionFactory( );

    static boost::shared_ptr< CostFunctionFactory > instance( void );

    ceres::CostFunction* generateCostFunction( const CameraConstPtr& camera,
                                               const Eigen::Vector3d& observed_P,
                                               const Eigen::Vector2d& observed_p,
                                               int flags ) const;

    private:
    static boost::shared_ptr< CostFunctionFactory > m_instance;
};
}

#endif
