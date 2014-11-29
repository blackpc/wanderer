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

Trajectory::Trajectory(double linearVelocity, double angularVelocity, double weight)
	: score_(0), weight_(weight), linearVelocity_(linearVelocity), angularVelocity_(angularVelocity) {
	path_ = nav_msgs::Path::Ptr(new nav_msgs::Path());
}

void Trajectory::addPosition(double x, double y) {
	geometry_msgs::PoseStamped newPose;

	newPose.header.stamp = ros::Time::now();
	newPose.pose.orientation.w = 1;
	newPose.pose.position.x = x;
	newPose.pose.position.y = y;

	path_->poses.push_back(newPose);
}

void Trajectory::addPosition(const tf::Vector3& position,
		const tf::Quaternion& orientation)
{
	geometry_msgs::PoseStamped newPose;

	// newPose.header.frame_id = ""; // set in getPath()
	newPose.header.stamp = ros::Time::now();

	newPose.pose.position.x = position.x();
	newPose.pose.position.y = position.y();
	newPose.pose.position.z = position.z();

	newPose.pose.orientation.x = orientation.x();
	newPose.pose.orientation.y = orientation.y();
	newPose.pose.orientation.z = orientation.z();
	newPose.pose.orientation.w = orientation.w();

	path_->poses.push_back(newPose);
}

nav_msgs::Path::Ptr Trajectory::getPath(bool updateStamp, const string& frameId) {
	path_->header.frame_id = frameId;

	foreach(geometry_msgs::PoseStamped& pose, path_->poses) {
		pose.header.stamp = ros::Time::now();
		pose.header.frame_id = frameId;
	}

	return path_;
}

nav_msgs::Path::Ptr Trajectory::getPath() const {
	return path_;
}

void Trajectory::clearPath() {
	path_->poses.clear();
}

void Trajectory::setScore(double score) {
	score_ = score;
}

double Trajectory::getScore() const {
	return score_;
}

double Trajectory::getLinearVelocity() const {
	return linearVelocity_;
}

void Trajectory::setWeight(double weight) {
}

double Trajectory::getWeight() const {
}

double Trajectory::getAngularVelocity() const {
	return angularVelocity_;
}
