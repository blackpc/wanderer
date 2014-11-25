/**
 * Filename: Trajectory.cpp
 *   Author: Igor Makhtes
 *     Date: Nov 25, 2014
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2014
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include <wanderer/Trajectory.h>

Trajectory::Trajectory() : score_(0) {
	path_.header.frame_id = "odom";
}

void Trajectory::addPosition(double x, double y) {
	geometry_msgs::PoseStamped newPose;


	newPose.header.frame_id = "odom";
	newPose.header.stamp = ros::Time::now();
	newPose.pose.orientation.w = 1;
	newPose.pose.position.x = x;
	newPose.pose.position.y = y;

	path_.poses.push_back(newPose);
}

void Trajectory::addPosition(const tf::Vector3& position,
		const tf::Quaternion& orientation)
{
	geometry_msgs::PoseStamped newPose;

	newPose.header.frame_id = "odom";
	newPose.header.stamp = ros::Time::now();

	newPose.pose.position.x = position.x();
	newPose.pose.position.y = position.y();
	newPose.pose.position.z = position.z();

	newPose.pose.orientation.x = orientation.x();
	newPose.pose.orientation.y = orientation.y();
	newPose.pose.orientation.z = orientation.z();
	newPose.pose.orientation.w = orientation.w();

	path_.poses.push_back(newPose);
}

nav_msgs::Path Trajectory::getPath(bool updateStamp) const {
	nav_msgs::Path path = path_;

	foreach(geometry_msgs::PoseStamped& pose, path.poses) {
		pose.header.stamp = ros::Time::now();
	}

	return path_;
}

void Trajectory::clearPath() {
	path_.poses.clear();
}

void Trajectory::setScore(double score) {
	score_ = score;
}

double Trajectory::getScore() const {
	return score_;
}
