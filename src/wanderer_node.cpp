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

#include <wanderer/TrajectorySimulator.h>
#include <wanderer/Wandering.h>


using namespace std;


int main(int argc, char **argv) {
	ros::init(argc, argv, "wanderer_node");
	ros::NodeHandle nodePrivate("~");


	Wandering wanderer;
	wanderer.start();


	TrajectorySimulator simulator(10, 0.1);
	ros::Publisher pathPublisher = nodePrivate.advertise<nav_msgs::Path>("/path", 1, false);

	double linear = 0.5;
	double angular = 0.1;

	double t = 0;

	while (ros::ok()) {

		for (int i = 0; i < 11; ++i) {
			Trajectory trajectory = simulator.simulate(linear, ((i - 5.0) / 10.0));

			pathPublisher.publish(trajectory.getPath(true));

//			t += 0.005;
		}

//		break;
		boost::this_thread::sleep(boost::posix_time::milliseconds(1000 / 60));
	}

	ros::spin();
	return 0;
}
