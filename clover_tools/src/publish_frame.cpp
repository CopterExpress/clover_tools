/*
 * Publish TF frame from PoseStamped or PoseWithCovarianceStamped topic
 * with optional inverse
 * Copyright (C) 2018 Copter Express Technologies
 *
 * Author: Oleg Kalachev <okalachev@gmail.com>
 *
 * Distributed under MIT License (available at https://opensource.org/licenses/MIT).
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 */

#include <string>
#include <ros/ros.h>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

using namespace geometry_msgs;

TransformStamped transform;
bool inverse;

inline TransformStamped inverseTransform(TransformStamped& transform)
{
	tf2::Transform t;
	tf2::fromMsg(transform.transform, t);
	auto result = transform;
	result.transform = tf2::toMsg(t.inverse());
	return result;
}

inline Pose getPose(const PoseStampedConstPtr& pose) { return pose->pose; }

inline Pose getPose(const PoseWithCovarianceStampedConstPtr& pose) { return pose->pose.pose; }

template <typename T>
void callback(const T& msg)
{
    static tf2_ros::TransformBroadcaster tf_broadcaster;

    transform.header.stamp = msg->header.stamp;
    transform.transform.rotation = getPose(msg).orientation;
    transform.transform.translation.x = getPose(msg).position.x;
    transform.transform.translation.y = getPose(msg).position.y;
    transform.transform.translation.z = getPose(msg).position.z;

    if (!inverse) {
        tf_broadcaster.sendTransform(transform);
    } else {
        tf_broadcaster.sendTransform(inverseTransform(transform));
    }
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "publish_frame");
	ros::NodeHandle nh, nh_priv("~");

	// Params
	nh_priv.param<std::string>("frame_id", transform.header.frame_id, "");
	nh_priv.param<std::string>("child_frame_id", transform.child_frame_id, "");
	nh_priv.param("inverse", inverse, false);

	auto pose_sub = nh_priv.subscribe<PoseStamped>("pose", 1, &callback);
	auto pose_cov_sub = nh_priv.subscribe<PoseWithCovarianceStamped>("pose_cov", 1, &callback);

	ROS_INFO("%s -> %s frame publisher: ready", transform.header.frame_id.c_str(), transform.child_frame_id.c_str());
	ros::spin();
}
