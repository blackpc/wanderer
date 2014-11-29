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

Wandering::Wandering(double minFrontDistance, double linearVelocity, double angularVelocity) {
	minFrontDistance_ = minFrontDistance;
	linearVelocity_ = linearVelocity;
	angularVelocity_ = angularVelocity;
}

void Wandering::spin() const {
	ros::NodeHandle nodePrivate("~");

	srand(time(0));

	LaserScanDataSource* laserScanDataSource = new LaserScanDataSource(nodePrivate, "/base_scan");
	CostMap costMap(laserScanDataSource, 0.25, 11, 11, 0.025, "base_link");
	ITrajectoryMatcher* trajectoryMatcher = new SimpleTrajectoryMatcher();

	/**
	 * Publishers
	 */
	ros::Publisher mapPublisher = nodePrivate.advertise<nav_msgs::OccupancyGrid>("/map", 1, false);
	ros::Publisher pathPublisher = nodePrivate.advertise<nav_msgs::Path>("/path", 10, false);
	ros::Publisher bestPathPublisher = nodePrivate.advertise<nav_msgs::Path>("/path_best", 1, false);
	ros::Publisher velocityPublisher = nodePrivate.advertise<geometry_msgs::Twist>("/cmd_vel", 1, false);

	const double simulationTime = minFrontDistance_ / linearVelocity_;
	TrajectorySimulator trajectorySimulator(simulationTime, 0.025);

	Trajectory::Ptr leftTrajectory = trajectorySimulator.simulate(1, 0);
	Trajectory::Ptr rightTrajectory = trajectorySimulator.simulate(1, 0);
	Trajectory::Ptr frontTrajectory = trajectorySimulator.simulate(linearVelocity_, 0);

	leftTrajectory->setWeight(0.1);
	rightTrajectory->setWeight(0.1);
	frontTrajectory->setWeight(1);

	leftTrajectory->rotate(M_PI_2);
	rightTrajectory->rotate(-M_PI_2);

	leftTrajectory->setVelocities(0, angularVelocity_);
	rightTrajectory->setVelocities(0, -angularVelocity_);

	ros::Rate rate(10);

	TrajectoryMatch::Ptr bestMatch;

	bool waitForFrontTrajectory = false;

	while (ros::ok()) {
		ros::spinOnce();

		/**
		 * Evaluate trajectories
		 */
		TrajectoryMatch::Ptr frontMatch = trajectoryMatcher->match(costMap, frontTrajectory);
		TrajectoryMatch::Ptr leftMatch = trajectoryMatcher->match(costMap, leftTrajectory);
		TrajectoryMatch::Ptr rightMatch = trajectoryMatcher->match(costMap, rightTrajectory);

		if (!waitForFrontTrajectory) {

			/**
			 * The robot is not turning right now so check all trajectories
			 */
			if (frontMatch->getScore() > leftMatch->getScore() &&
					frontMatch->getScore() > rightMatch->getScore()) {
				/**
				 * Best match is the front
				 */
				bestMatch = frontMatch;
			} else {

				/**
				 * Front is blocked, select best match from right or left
				 */
				if (leftMatch->getScore() > rightMatch->getScore())
					bestMatch = leftMatch;
				else
					bestMatch = rightMatch;

				waitForFrontTrajectory = true;
			}

		} else {
			/**
			 * Front was blocked previously, so we'r turning in place to find an open front trajectory
			 */
			if (frontMatch->getScore() > 0) {
				/**
				 * Check front trajectory, if free, drive straight
				 */
				bestMatch = frontMatch;
				waitForFrontTrajectory = false;
			}
		}


		/**
		 * Publish all paths
		 */
		pathPublisher.publish(leftTrajectory->getPath(true, "base_link"));
		pathPublisher.publish(rightTrajectory->getPath(true, "base_link"));
		pathPublisher.publish(frontTrajectory->getPath(true, "base_link"));

		/**
		 * Publish best matched trajectory
		 */
		bestPathPublisher.publish(bestMatch->getTrajectory()->getPath(true, "base_link"));

		/**
		 * Publish velocity command
		 */
		velocityPublisher.publish(bestMatch->getTrajectory()->getTwistMessage());

		/**
		 * Publish local cost map
		 */
		mapPublisher.publish(costMap.getOccupancyGrid());

		rate.sleep();
	}

	delete laserScanDataSource;
}

Trajectory::VectorPtr Wandering::createTrajectories(double simulationTime, double granularity) const {
	TrajectorySimulator trajectorySimulator(simulationTime, granularity);
	Trajectory::VectorPtr trajectories(new Trajectory::Vector());

	Trajectory::Ptr trajectory;

	trajectory = trajectorySimulator.simulate(0.0, -angularVelocity_);
	trajectory->setWeight(0.5);
	trajectories->push_back(trajectory);

//	trajectory = trajectorySimulator.simulate(0.3, -0.25);
//	trajectory->setWeight(0.75);
//	trajectories->push_back(trajectory);

	trajectory = trajectorySimulator.simulate(linearVelocity_, 0);
	trajectory->setWeight(1.0);
	trajectories->push_back(trajectory);

//	trajectory = trajectorySimulator.simulate(0.3, 0.25);
//	trajectory->setWeight(0.75);
//	trajectories->push_back(trajectory);

	trajectory = trajectorySimulator.simulate(0.0, angularVelocity_);
	trajectory->setWeight(0.5);
	trajectories->push_back(trajectory);

	return trajectories;
}
