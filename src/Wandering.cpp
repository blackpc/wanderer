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

	Trajectory::VectorPtr trajectories = createTrajectories(2, 0.1);

	ros::Rate rate(10);

	TrajectoryMatch::Ptr bestMatch;

	while (ros::ok()) {
		ros::spinOnce();

		/**
		 * Evaluate trajectories
		 */
		TrajectoryMatch::SetPtr matches = trajectoryMatcher->match(costMap, trajectories);

		bestMatch = *matches->begin();

		/**
		 * Publish all paths
		 */
		for (int i = 0; i < trajectories->size(); ++i)
			pathPublisher.publish((*trajectories)[i]->getPath(true, "base_link"));

		/**
		 * Publish best matched trajectory
		 */
		bestPathPublisher.publish(bestMatch->getTrajectory()->getPath(true, "base_link"));

		/**
		 * Publish velocity command
		 */
		velocityPublisher.publish(bestMatch->getTrajectory()->getMotionModelAs<SkidSteerModel>()->getTwistMessage());

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

	trajectory = trajectorySimulator.simulate(new SkidSteerModel(0.3, -1.0));
	trajectory->setWeight(0.4);
	trajectories->push_back(trajectory);

	trajectory = trajectorySimulator.simulate(new SkidSteerModel(0.5, -1.0));
	trajectory->setWeight(0.5);
	trajectories->push_back(trajectory);

	trajectory = trajectorySimulator.simulate(new SkidSteerModel(0.5, -0.5));
	trajectory->setWeight(0.75);
	trajectories->push_back(trajectory);

	trajectory = trajectorySimulator.simulate(new SkidSteerModel(0.5, 0));
	trajectory->setWeight(1.0);
	trajectories->push_back(trajectory);

	trajectory = trajectorySimulator.simulate(new SkidSteerModel(0.5, 0.5));
	trajectory->setWeight(0.75);
	trajectories->push_back(trajectory);

	trajectory = trajectorySimulator.simulate(new SkidSteerModel(0.5, 1.0));
	trajectory->setWeight(0.5);
	trajectories->push_back(trajectory);

	trajectory = trajectorySimulator.simulate(new SkidSteerModel(0.3, 1.0));
	trajectory->setWeight(0.4);
	trajectories->push_back(trajectory);

	return trajectories;
}
