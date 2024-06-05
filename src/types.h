#pragma once

#ifdef VNOID_BUILD_CNOID
# include <cnoid/EigenTypes>
#else
# include <Eigen/Core>
# include <Eigen/Geometry>
 typedef Eigen::Matrix2d Matrix2;
 typedef Eigen::Vector2d Vector2;
 typedef Eigen::Matrix3d Matrix3;
 typedef Eigen::Vector3d Vector3;
 typedef Eigen::AngleAxisd AngleAxis;
 typedef Eigen::Quaterniond Quaternion;
#endif
