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
    randomSteer_ = false;
    randomSteerTime_ = ros::Time::now();
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

	TrajectoryMatch::Ptr rightInPlaceMatch =
	        trajectoryMatcher_->match(costMap, rightInPlaceTrajectory_);

	TrajectoryMatch::Ptr leftInPlaceMatch =
	        trajectoryMatcher_->match(costMap, leftInPlaceTrajectory_);

	TrajectoryMatch::SetPtr tempSet(new TrajectoryMatch::Set());

	bool frontBlocked = false;

	if (frontMatch->isBlocked() && leftMatches->begin()->get()->isBlocked() && rightMatches->begin()->get()->isBlocked())
	    frontBlocked = true;

	if ( (ros::Time::now() - preferSideChangeTime_).toSec() > 60) {
		preferSideChangeTime_ = ros::Time::now();
		preferRight_ = !preferRight_;
	}

	if (randomSteerTime_ < ros::Time::now()) {
	    ros::Time nextRandomSteerUpdate =
	            ros::Time::now() + (randomSteer_ ? ros::Duration(40) : ros::Duration(15) );
	    randomSteerTime_ = nextRandomSteerUpdate;
	    randomSteer_ = !randomSteer_;
	    ROS_INFO("Random steer = %s, Prefer right = %s", randomSteer_ ? "True" : "False", preferRight_ ? "True" : "False");
	}


	if (frontBlocked) {
	    return rightInPlaceMatch;
	}
	else if (!frontMatch->isBlocked() &&
	        !leftMatches->begin()->get()->isBlocked() &&
	        !rightMatches->begin()->get()->isBlocked())
	{
	    // All front trajectories are clear
	    if (randomSteer_)
	        if (preferRight_)
	            return *rightMatches->begin();
	        else
	            return *leftMatches->begin();

	    return frontMatch;
	}
	else if (!frontMatch->isBlocked()) {
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
				!rightMatches->begin()->get()->isBlocked() &&
				leftMatches->begin()->get()->getScore() ==
                rightMatches->begin()->get()->getScore())
		{
			if (preferRight_)
				return *rightMatches->begin();
			else
				return *leftMatches->begin();
		}
		tempSet->insert(*rightMatches->begin());
		tempSet->insert(*leftMatches->begin());

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

	createTrajectories(2.5, 0.2);

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

		for (int i = 0; i < leftTrajectories_->size(); ++i) {
			pathPublisher.publish((*leftTrajectories_)[i]->getPath(true, baseFrameId_));
 		}

		for (int i = 0; i < rightTrajectories_->size(); ++i) {
			pathPublisher.publish((*rightTrajectories_)[i]->getPath(true, baseFrameId_));
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

	Trajectory::Ptr trajectory;

	/**
	 * Front
	 */
	frontTrajectory_ = trajectorySimulator.simulate(new AckermannModel(0.2, 0.5, 0));
	frontTrajectory_->setWeight(1.0);

	/**
	 * Left
	 */
	trajectory = trajectorySimulator.simulate(new AckermannModel(0.2, 0.5, 0.161799388));
	trajectory->setWeight(0.5);
	leftTrajectories_->push_back(trajectory);

	trajectory = trajectorySimulator.simulate(new AckermannModel(0.2, 0.5, 0.261799388));
	trajectory->setWeight(0.4);
	leftTrajectories_->push_back(trajectory);

	/**
	 * Right
	 */
	trajectory = trajectorySimulator.simulate(new AckermannModel(0.2, 0.5, -0.261799388));
	trajectory->setWeight(0.4);
	rightTrajectories_->push_back(trajectory);

	trajectory = trajectorySimulator.simulate(new AckermannModel(0.2, 0.5, -0.161799388));
	trajectory->setWeight(0.5);
	rightTrajectories_->push_back(trajectory);

	/**
	 * In place
	 */
	rightInPlaceTrajectory_ = trajectorySimulator.simulate(new AckermannModel(0.2, 0.0, -1.0));
	rightInPlaceTrajectory_->setWeight(1.0);

	leftInPlaceTrajectory_ = trajectorySimulator.simulate(new AckermannModel(0.2, 0.0, 1.0));
	leftInPlaceTrajectory_->setWeight(1.0);
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
