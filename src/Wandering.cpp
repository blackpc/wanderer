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
	  enabled_(enabled), publishStop_(false) { }

void Wandering::spin() {
	ros::NodeHandle nodePrivate("~");

	srand(time(0));

	LaserScanDataSource* laserScanDataSource = new LaserScanDataSource(nodePrivate, "/scan");
	CostMap costMap(laserScanDataSource, 0.25, 3, 3, 0.025, baseFrameId_);
	ITrajectoryMatcher* trajectoryMatcher = new SimpleTrajectoryMatcher();

	/**
	 * Publishers
	 */
	ros::Publisher mapPublisher = nodePrivate.advertise<nav_msgs::OccupancyGrid>("costmap", 1, false);
	ros::Publisher pathPublisher = nodePrivate.advertise<nav_msgs::Path>("path", 1000, false);
	ros::Publisher bestPathPublisher = nodePrivate.advertise<nav_msgs::Path>("path_best", 1, false);
//	ros::Publisher velocityPublisher = nodePrivate.advertise<geometry_msgs::Twist>("/cmd_vel", 1, false);
	ros::Publisher ackermannPublisher = nodePrivate.advertise<ackermann_msgs::AckermannDriveStamped>("/ackermann_cmd", 1, false);
	ros::Subscriber stateSubscriber = nodePrivate.subscribe(string("/decision_making/" + robotId_ + "/events"), 10, &Wandering::stateCallback, this);

	Trajectory::VectorPtr trajectories = createTrajectories(0.75, 0.1);

	ros::Rate rate(10);

	TrajectoryMatch::Ptr bestMatch;

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
		TrajectoryMatch::SetPtr matches = trajectoryMatcher->match(costMap, trajectories);

		bestMatch = *matches->begin();

		/**
		 * Publish all paths
		 */
		for (int i = 0; i < trajectories->size(); ++i) {
			pathPublisher.publish((*trajectories)[i]->getPath(true, baseFrameId_));
			boost::this_thread::sleep(boost::posix_time::milliseconds(1));
		}

		/**
		 * Publish best matched trajectory
		 */
		bestPathPublisher.publish(bestMatch->getTrajectory()->getPath(true, baseFrameId_));

		/**
		 * Publish velocity command
		 */
//		velocityPublisher.publish(bestMatch->getTrajectory()->getMotionModelAs<SkidSteerModel>()->getTwistMessage());
		ackermannPublisher.publish(bestMatch->getTrajectory()->getMotionModelAs<AckermannModel>()->getAckermannMessage());

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

	int trajectoriesCount = 7;
	/**
	 * Skid steer trajectories
	 */
//	for (int i = 0; i < trajectoriesCount + 1; ++i) {
//		trajectory = trajectorySimulator.simulate(new SkidSteerModel(0.5, -0.785398163 + (1.785398163 / ((trajectoriesCount + 1) / 2)) * i));
//		trajectory->setWeight( 1 - fabs(i - (trajectoriesCount / 2.0)) / (trajectoriesCount / 2.0) );
//		trajectories->push_back(trajectory);
//
//		trajectory = trajectorySimulator.simulate(new SkidSteerModel(-0.5, -0.785398163 + (1.785398163 / ((trajectoriesCount + 1) / 2)) * i));
//		trajectory->setWeight( 0.5 * (1 - fabs(i - (trajectoriesCount / 2.0)) / (trajectoriesCount / 2.0)) );
//		trajectories->push_back(trajectory);
//	}

	/**
	 * Ackermann trajectories
	 */
//	for (int i = 0; i < 102; ++i) {
//		trajectory = trajectorySimulator.simulate(new AckermannModel(0.2, 0.5, -0.785398163 + (0.785398163 / 50) * i));
//		trajectory->setWeight( 1 - fabs(i - 50.5) / 50.5 );
//		trajectories->push_back(trajectory);
//
//		trajectory = trajectorySimulator.simulate(new AckermannModel(0.2, -0.5, -0.785398163 + (0.785398163 / 50) * i));
//		trajectory->setWeight( 0.5 * (1 - fabs(i - 50.5) / 50.5) );
//		trajectories->push_back(trajectory);
//	}

	trajectory = trajectorySimulator.simulate(new AckermannModel(0.2, 0.5, -0.785398163));
	trajectory->setWeight(0.4);
	trajectories->push_back(trajectory);

	trajectory = trajectorySimulator.simulate(new AckermannModel(0.2, 0.3, -0.785398163));
	trajectory->setWeight(0.39);
	trajectories->push_back(trajectory);

	trajectory = trajectorySimulator.simulate(new AckermannModel(0.2, 0.2, -0.785398163));
	trajectory->setWeight(0.38);
	trajectories->push_back(trajectory);

	trajectory = trajectorySimulator.simulate(new AckermannModel(0.2, 0.1, -0.785398163));
	trajectory->setWeight(0.38);
	trajectories->push_back(trajectory);

	trajectory = trajectorySimulator.simulate(new AckermannModel(0.2, 0.5, -0.5));
	trajectory->setWeight(0.5);
	trajectories->push_back(trajectory);

	trajectory = trajectorySimulator.simulate(new AckermannModel(0.2, 0.5, -0.3));
	trajectory->setWeight(0.75);
	trajectories->push_back(trajectory);

	trajectory = trajectorySimulator.simulate(new AckermannModel(0.2, 0.5, 0));
	trajectory->setWeight(1.0);
	trajectories->push_back(trajectory);

	trajectory = trajectorySimulator.simulate(new AckermannModel(0.2, 0.5, 0.3));
	trajectory->setWeight(0.75);
	trajectories->push_back(trajectory);

	trajectory = trajectorySimulator.simulate(new AckermannModel(0.2, 0.5, 0.5));
	trajectory->setWeight(0.5);
	trajectories->push_back(trajectory);


	trajectory = trajectorySimulator.simulate(new AckermannModel(0.2, 0.1, 0.785398163));
	trajectory->setWeight(0.38);
	trajectories->push_back(trajectory);

	trajectory = trajectorySimulator.simulate(new AckermannModel(0.2, 0.2, 0.785398163));
	trajectory->setWeight(0.38);
	trajectories->push_back(trajectory);

	trajectory = trajectorySimulator.simulate(new AckermannModel(0.2, 0.3, 0.785398163));
	trajectory->setWeight(0.39);
	trajectories->push_back(trajectory);

	trajectory = trajectorySimulator.simulate(new AckermannModel(0.2, 0.5, 0.785398163));
	trajectory->setWeight(0.4);
	trajectories->push_back(trajectory);



	trajectory = trajectorySimulator.simulate(new AckermannModel(0.2, -0.3, -0.785398163));
	trajectory->setWeight(0.04);
	trajectories->push_back(trajectory);

	trajectory = trajectorySimulator.simulate(new AckermannModel(0.2, -0.1, -0.785398163));
	trajectory->setWeight(0.04);
	trajectories->push_back(trajectory);

	trajectory = trajectorySimulator.simulate(new AckermannModel(0.2, -0.3, 0));
	trajectory->setWeight(0.05);
	trajectories->push_back(trajectory);

	trajectory = trajectorySimulator.simulate(new AckermannModel(0.2, -0.1, 0.785398163));
	trajectory->setWeight(0.06);
	trajectories->push_back(trajectory);

	trajectory = trajectorySimulator.simulate(new AckermannModel(0.2, -0.3, 0.785398163));
	trajectory->setWeight(0.06);
	trajectories->push_back(trajectory);

	return trajectories;
}

void Wandering::stateCallback(const std_msgs::String::Ptr& message) {
	if (message->data == "RESUME") {
		enabled_ = true;
		publishStop_ = false;
	} else if (message->data == "PAUSE") {
		publishStop_ = true;
		enabled_ = false;
	}
}
