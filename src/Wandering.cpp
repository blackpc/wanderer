/**
 * Filename: Wandering.cpp
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

#include <wanderer/Wandering.h>

Wandering::Wandering(const string& robotId,
		const string& baseFrameId, bool enabled)
	: robotId_(robotId), baseFrameId_(baseFrameId),
	  enabled_(enabled), publishStop_(false), preferRight_(true)
{
	trajectoryMatcher_ = new SimpleTrajectoryMatcher();
	preferSideChangeTime_ = ros::Time::now();
}


Wandering::~Wandering() {
	delete trajectoryMatcher_;
}

TrajectoryMatch::Ptr Wandering::chooseBestTrajectory(CostMap& costMap) {

	TrajectoryMatch::Ptr frontMatch =
			trajectoryMatcher_->match(costMap, frontTrajectory_);

	TrajectoryMatch::SetPtr leftMatches =
			trajectoryMatcher_->match(costMap, leftTrajectories_);

	TrajectoryMatch::SetPtr rightMatches =
			trajectoryMatcher_->match(costMap, rightTrajectories_);

	TrajectoryMatch::SetPtr rearLeftMatches =
			trajectoryMatcher_->match(costMap, rearLeftTrajectories_);

	TrajectoryMatch::SetPtr rearRightMatches =
			trajectoryMatcher_->match(costMap, rearRightTrajectories_);

	TrajectoryMatch::Ptr rearMatch =
			trajectoryMatcher_->match(costMap, rearTrajectory_);

	TrajectoryMatch::SetPtr tempSet(new TrajectoryMatch::Set());

	if ( (ros::Time::now() - preferSideChangeTime_).toSec() > 30) {
		preferSideChangeTime_ = ros::Time::now();
		preferRight_ = !preferRight_;
	}

	if (!frontMatch->isBlocked()) {
		/**
		 * Front is free, choose it
		 */
		return frontMatch;

	} else {

		/**
		 * Both left and right are not blocked,
		 * so choose preferable side
		 */
		if (!leftMatches->begin()->get()->isBlocked() &&
				!rightMatches->begin()->get()->isBlocked()) {
			if (preferRight_)
				return *rightMatches->begin();
			else
				return *leftMatches->begin();
		}

		tempSet->insert(rearMatch);
		tempSet->insert(*rightMatches->begin());
		tempSet->insert(*leftMatches->begin());

		if (rearLeftMatches->begin()->get()->getScore() ==
				rearRightMatches->begin()->get()->getScore()) {

			/**
			 * If front right is preferable, then choose rear left to
			 * turn in place
			 */
			if (preferRight_)
				tempSet->insert(*rearLeftMatches->begin());
			else
				tempSet->insert(*rearRightMatches->begin());
		} else {
			tempSet->insert(*rearLeftMatches->begin());
			tempSet->insert(*rearRightMatches->begin());
		}


		return *tempSet->begin();
	}
}

void Wandering::spin() {
	ros::NodeHandle nodePrivate("~");

	srand(time(0));

	LaserScanDataSource* laserScanDataSource = new LaserScanDataSource(nodePrivate, "/scan");
	CostMap costMap(laserScanDataSource, new RosParametersProvider());

	/**
	 * Publishers
	 */
	ros::Publisher mapPublisher = nodePrivate.advertise<nav_msgs::OccupancyGrid>("costmap", 1, false);
	ros::Publisher pathPublisher = nodePrivate.advertise<nav_msgs::Path>("path", 1, false);
	ros::Publisher bestPathPublisher = nodePrivate.advertise<nav_msgs::Path>("path_best", 1, false);
	ros::Publisher ackermannPublisher = nodePrivate.advertise<ackermann_msgs::AckermannDriveStamped>("/ackermann_cmd", 1, false);
	ros::Subscriber stateSubscriber = nodePrivate.subscribe(string("/decision_making/" + robotId_ + "/events"), 1, &Wandering::stateCallback, this);

	createTrajectories(0.75, 0.1);

	ros::Rate rate(5);

	while (ros::ok()) {
		ros::spinOnce();

		if (!enabled_) {

			if (publishStop_) {
				ackermannPublisher.publish(ackermann_msgs::AckermannDriveStamped());
				publishStop_ = 0;
			}

			rate.sleep();
			continue;
		}

		/**
		 * Evaluate trajectories
		 */
		TrajectoryMatch::Ptr bestMatch = chooseBestTrajectory(costMap);

		/**
		 * Publish all paths
		 */
		pathPublisher.publish(frontTrajectory_->getPath(true, baseFrameId_));
		pathPublisher.publish(rearTrajectory_->getPath(true, baseFrameId_));

		for (int i = 0; i < leftTrajectories_->size(); ++i) {
			pathPublisher.publish((*leftTrajectories_)[i]->getPath(true, baseFrameId_));
			boost::this_thread::sleep(boost::posix_time::milliseconds(1));
		}

		for (int i = 0; i < rightTrajectories_->size(); ++i) {
			pathPublisher.publish((*rightTrajectories_)[i]->getPath(true, baseFrameId_));
			boost::this_thread::sleep(boost::posix_time::milliseconds(1));
		}

		for (int i = 0; i < rearLeftTrajectories_->size(); ++i) {
			pathPublisher.publish((*rearLeftTrajectories_)[i]->getPath(true, baseFrameId_));
			boost::this_thread::sleep(boost::posix_time::milliseconds(1));
		}

		for (int i = 0; i < rearRightTrajectories_->size(); ++i) {
			pathPublisher.publish((*rearLeftTrajectories_)[i]->getPath(true, baseFrameId_));
			boost::this_thread::sleep(boost::posix_time::milliseconds(1));
		}

		/**
		 * Publish best matched trajectory
		 */
		bestPathPublisher.publish(bestMatch->getTrajectory()->getPath(true, baseFrameId_));

		/**
		 * Publish velocity command
		 */
		ackermannPublisher.publish(bestMatch->getTrajectory()->getMotionModelAs<AckermannModel>()->getAckermannMessage());

		/**
		 * Publish local cost map
		 */
		mapPublisher.publish(costMap.getOccupancyGrid());

		rate.sleep();
	}

}

void Wandering::createTrajectories(double simulationTime, double granularity) {
	TrajectorySimulator trajectorySimulator(simulationTime, granularity);

	leftTrajectories_  = Trajectory::VectorPtr(new Trajectory::Vector());
	rightTrajectories_ = Trajectory::VectorPtr(new Trajectory::Vector());
	rearLeftTrajectories_  = Trajectory::VectorPtr(new Trajectory::Vector());
	rearRightTrajectories_  = Trajectory::VectorPtr(new Trajectory::Vector());

	Trajectory::Ptr trajectory;

	/**
	 * Front
	 */
	frontTrajectory_ = trajectorySimulator.simulate(new AckermannModel(0.2, 0.5, 0));
	frontTrajectory_->setWeight(1.0);

	/**
	 * Left
	 */
	trajectory = trajectorySimulator.simulate(new AckermannModel(0.2, 0.5, 0.3));
	trajectory->setWeight(0.75);
	leftTrajectories_->push_back(trajectory);

	trajectory = trajectorySimulator.simulate(new AckermannModel(0.2, 0.5, 0.5));
	trajectory->setWeight(0.5);
	leftTrajectories_->push_back(trajectory);

	trajectory = trajectorySimulator.simulate(new AckermannModel(0.2, 0.35, 0.785398163));
	trajectory->setWeight(0.38);
	leftTrajectories_->push_back(trajectory);

	trajectory = trajectorySimulator.simulate(new AckermannModel(0.2, 0.40, 0.785398163));
	trajectory->setWeight(0.38);
	leftTrajectories_->push_back(trajectory);

	trajectory = trajectorySimulator.simulate(new AckermannModel(0.2, 0.45, 0.785398163));
	trajectory->setWeight(0.39);
	leftTrajectories_->push_back(trajectory);

	trajectory = trajectorySimulator.simulate(new AckermannModel(0.2, 0.5, 0.785398163));
	trajectory->setWeight(0.4);
	leftTrajectories_->push_back(trajectory);

	/**
	 * Right
	 */
	trajectory = trajectorySimulator.simulate(new AckermannModel(0.2, 0.5, -0.785398163));
	trajectory->setWeight(0.4);
	rightTrajectories_->push_back(trajectory);

	trajectory = trajectorySimulator.simulate(new AckermannModel(0.2, 0.45, -0.785398163));
	trajectory->setWeight(0.39);
	rightTrajectories_->push_back(trajectory);

	trajectory = trajectorySimulator.simulate(new AckermannModel(0.2, 0.40, -0.785398163));
	trajectory->setWeight(0.38);
	rightTrajectories_->push_back(trajectory);

	trajectory = trajectorySimulator.simulate(new AckermannModel(0.2, 0.35, -0.785398163));
	trajectory->setWeight(0.38);
	rightTrajectories_->push_back(trajectory);

	trajectory = trajectorySimulator.simulate(new AckermannModel(0.2, 0.5, -0.5));
	trajectory->setWeight(0.5);
	rightTrajectories_->push_back(trajectory);

	trajectory = trajectorySimulator.simulate(new AckermannModel(0.2, 0.5, -0.3));
	trajectory->setWeight(0.75);
	rightTrajectories_->push_back(trajectory);

	/**
	 * Rear
	 */
	trajectory = trajectorySimulator.simulate(new AckermannModel(0.2, -0.45, -0.785398163));
	trajectory->setWeight(0.04);
	rearRightTrajectories_->push_back(trajectory);

	trajectory = trajectorySimulator.simulate(new AckermannModel(0.2, -0.5, -0.785398163));
	trajectory->setWeight(0.04);
	rearRightTrajectories_->push_back(trajectory);

	rearTrajectory_ = trajectorySimulator.simulate(new AckermannModel(0.35, -0.4, 0));
	rearTrajectory_->setWeight(0.05);

	trajectory = trajectorySimulator.simulate(new AckermannModel(0.2, -0.5, 0.785398163));
	trajectory->setWeight(0.06);
	rearLeftTrajectories_->push_back(trajectory);

	trajectory = trajectorySimulator.simulate(new AckermannModel(0.2, -0.45, 0.785398163));
	trajectory->setWeight(0.06);
	rearLeftTrajectories_->push_back(trajectory);

}

void Wandering::stateCallback(const std_msgs::String::Ptr& message) {
	if (message->data == "RESUME") {
		enabled_ = true;
		publishStop_ = false;
		ROS_INFO("Started!");
	} else if (message->data == "PAUSE") {

		if (enabled_)
			publishStop_ = true;

		enabled_ = false;
		ROS_INFO("Stoped!");
	}
}
