// Implementation
#include "graph_msf_ros/util/conversions.h"

namespace graph_msf {

Eigen::Matrix<double, 6, 6> convertCovarianceGtsamConventionToRosConvention(const Eigen::Matrix<double, 6, 6>& covGtsam) {
  Eigen::Matrix<double, 6, 6> covRos;
  covRos.setZero();
  covRos.block<3, 3>(0, 0) = covGtsam.block<3, 3>(3, 3);
  covRos.block<3, 3>(3, 3) = covGtsam.block<3, 3>(0, 0);
  covRos.block<3, 3>(0, 3) = covGtsam.block<3, 3>(3, 0);
  covRos.block<3, 3>(3, 0) = covGtsam.block<3, 3>(0, 3);
  return covRos;
}

void geometryPoseToEigen(const geometry_msgs::PoseWithCovarianceStamped& odomLidar, Eigen::Matrix4d& T) {
  tf::Quaternion tf_q;
  tf::quaternionMsgToTF(odomLidar.pose.pose.orientation, tf_q);
  Eigen::Vector3d t(odomLidar.pose.pose.position.x, odomLidar.pose.pose.position.y, odomLidar.pose.pose.position.z);
  Eigen::Quaternion<double> q(tf_q.getW(), tf_q.getX(), tf_q.getY(), tf_q.getZ());
  T.setIdentity();
  T.block<3, 3>(0, 0) = q.matrix();
  T.block<3, 1>(0, 3) = t;
}

void odomMsgToEigen(const nav_msgs::Odometry& odomLidar, Eigen::Matrix4d& T) {
  tf::Quaternion tf_q;
  tf::quaternionMsgToTF(odomLidar.pose.pose.orientation, tf_q);
  Eigen::Vector3d t(odomLidar.pose.pose.position.x, odomLidar.pose.pose.position.y, odomLidar.pose.pose.position.z);
  Eigen::Quaternion<double> q(tf_q.getW(), tf_q.getX(), tf_q.getY(), tf_q.getZ());
  T.setIdentity();
  T.block<3, 3>(0, 0) = q.matrix();
  T.block<3, 1>(0, 3) = t;
}

void odomMsgToTf(const nav_msgs::Odometry& odomLidar, tf::Transform& T) {
  tf::Quaternion tf_q;
  tf::quaternionMsgToTF(odomLidar.pose.pose.orientation, tf_q);
  tf::Vector3 tf_t(odomLidar.pose.pose.position.x, odomLidar.pose.pose.position.y, odomLidar.pose.pose.position.z);
  T = tf::Transform(tf_q, tf_t);
}

tf::Transform matrix3ToTf(const Eigen::Matrix3d& R) {
  Eigen::Quaterniond q(R);
  tf::Transform tf_R;
  tf_R.setRotation(tf::Quaternion(q.x(), q.y(), q.z(), q.w()));
  tf_R.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
  return tf_R;
}

tf::Transform matrix4ToTf(const Eigen::Matrix4d& T) {
  Eigen::Quaterniond q(T.block<3, 3>(0, 0));
  tf::Transform tf_T;
  tf_T.setRotation(tf::Quaternion(q.x(), q.y(), q.z(), q.w()));
  tf_T.setOrigin(tf::Vector3(T(0, 3), T(1, 3), T(2, 3)));
  return tf_T;
}

tf::Transform isometry3ToTf(const Eigen::Isometry3d& T) {
  Eigen::Quaterniond q(T.rotation());
  tf::Transform tf_T;
  tf_T.setRotation(tf::Quaternion(q.x(), q.y(), q.z(), q.w()));
  tf_T.setOrigin(tf::Vector3(T(0, 3), T(1, 3), T(2, 3)));
  return tf_T;
}

void tfToMatrix4(const tf::Transform& tf_T, Eigen::Matrix4d& T) {
  tf::Quaternion tf_q = tf_T.getRotation();
  Eigen::Quaternion<double> q(tf_q.getW(), tf_q.getX(), tf_q.getY(), tf_q.getZ());
  Eigen::Vector3d t(tf_T.getOrigin().getX(), tf_T.getOrigin().getY(), tf_T.getOrigin().getZ());
  T.setIdentity();
  T.block<3, 3>(0, 0) = q.matrix();
  T.block<3, 1>(0, 3) = t;
}

void tfToIsometry3(const tf::Transform& tf_T, Eigen::Isometry3d& T) {
  tf::Quaternion tf_q = tf_T.getRotation();
  Eigen::Quaternion<double> q(tf_q.getW(), tf_q.getX(), tf_q.getY(), tf_q.getZ());
  Eigen::Vector3d t(tf_T.getOrigin().getX(), tf_T.getOrigin().getY(), tf_T.getOrigin().getZ());
  T.setIdentity();
  T.rotate(q);
  T.pretranslate(t);
}

tf::Transform pose3ToTf(const Eigen::Matrix3d& T) {
  Eigen::Quaterniond q(T);
  tf::Transform tf_T;
  tf_T.setRotation(tf::Quaternion(q.x(), q.y(), q.z(), q.w()));
  tf_T.setOrigin(tf::Vector3(T(0, 3), T(1, 3), T(2, 3)));
  return tf_T;
}

}  // namespace graph_msf
