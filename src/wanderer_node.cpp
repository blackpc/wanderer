/**
 * Filename: wanderer_node.cpp
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


#include <boost/thread.hpp>

#include <wanderer/CostMap.h>
#include <wanderer/LaserScanDataSource.h>
#include <wanderer/TrajectorySimulator.h>
#include <wanderer/Wandering.h>


using namespace std;
using namespace cv;

void test() {
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "wanderer_node");
	ros::NodeHandle nodePrivate("~");

	LaserScanDataSource* laserScanDataSource = new LaserScanDataSource(nodePrivate, "/base_scan");
	CostMap costMap(laserScanDataSource, 0.25, 11, 11, 0.025, "base_link");
	TrajectorySimulator trajectorySimulator(2, 0.1);

	ros::Publisher mapPublisher = nodePrivate.advertise<nav_msgs::OccupancyGrid>("/map", 1, false);
	ros::Publisher pathPublisher = nodePrivate.advertise<nav_msgs::Path>("/path", 10, false);

	ros::Rate rate(10);

	while (ros::ok()) {
		ros::spinOnce();

		mapPublisher.publish(costMap.getOccupancyGrid());

		for (int i = 0; i < 11; ++i) {
			Trajectory::Ptr trajectory = trajectorySimulator.simulate(1, pow( 2 * (i - 5.0) / 11.0, 3));
			pathPublisher.publish(trajectory->getPath(true, "base_link"));
			boost::this_thread::sleep(boost::posix_time::milliseconds(1));
		}

		rate.sleep();
	}

	delete laserScanDataSource;
	return 0;
}
