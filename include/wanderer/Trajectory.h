/**
 * Filename: Trajectory.h
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

#ifndef INCLUDE_WANDERER_TRAJECTORY_H_
#define INCLUDE_WANDERER_TRAJECTORY_H_


#include <vector>

#include <boost/foreach.hpp>

#include <nav_msgs/Path.h>
#include <tf/tf.h>


using namespace std;


#define foreach BOOST_FOREACH


class Trajectory;

typedef vector<Trajectory> Trajectories;

/*
 *
 */
class Trajectory {

public:

	Trajectory();

public:

	void addPosition(double x, double y);
	void addPosition(const tf::Vector3& position, const tf::Quaternion& orientation);
	nav_msgs::Path getPath(bool updateStamp = false) const;
	void clearPath();

	void setScore(double score);
	double getScore() const;

private:

	double score_;
	nav_msgs::Path path_;


};

#endif /* INCLUDE_WANDERER_TRAJECTORY_H_ */
