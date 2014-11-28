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


int main(int argc, char **argv) {
	ros::init(argc, argv, "wanderer_node");
	ros::NodeHandle nodePrivate("~");

	LaserScanDataSource* laserScanDataSource = new LaserScanDataSource(nodePrivate, "/base_scan");
	CostMap m(laserScanDataSource, 100, 100, 0.1, "odom");

	ros::Publisher mapPublisher = nodePrivate.advertise<nav_msgs::OccupancyGrid>("/map", 1, false);

	ros::Rate rate(10);

	int counter = 10 * 5;

	while (counter-- > 0) {
		ros::spinOnce();

		mapPublisher.publish(m.getOccupancyGrid());
		rate.sleep();

		cout << counter << endl;
	}

	delete laserScanDataSource;
	return 0;
}
