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
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>


using namespace std;


#define foreach BOOST_FOREACH


/*
 * Represents a single simulated path
 */
class Trajectory {

public:

	typedef boost::shared_ptr<Trajectory> Ptr;
	typedef boost::shared_ptr<Trajectory const> ConstPtr;
	typedef vector<Trajectory::Ptr> Vector;
	typedef boost::shared_ptr<Vector> VectorPtr;

public:

	Trajectory(double linearVelocity, double angularVelocity, double weight = 1);

public:

	/**
	 * Adds position to the path with zero rotation
	 * @param x
	 * @param y
	 */
	void addPosition(double x, double y);

	/**
	 * Adds position to the path
	 * @param position
	 * @param orientation
	 */
	void addPosition(const tf::Vector3& position, const tf::Quaternion& orientation);

	/**
	 * Returns path with updated time stamp and frame id
	 * @param updateStamp
	 * @param frameId
	 * @return
	 */
	nav_msgs::Path::Ptr getPath(
			bool updateStamp, const string& frameId);

	/**
	 * Returns path with a default frame id and old time stamp
	 * @return
	 */
	inline nav_msgs::Path::Ptr getPath() const {
		return path_;
	}

	/**
	 * Removes all points from the path
	 */
	inline void clearPath() {
		path_->poses.clear();
	}

	/**
	 * Sets weight
	 * @param weight Value in range [0, 1]
	 */
	void setWeight(double weight);

	/**
	 * Gets the weight
	 * @return
	 */
	inline double getWeight() const {
		return weight_;
	}

	inline double getLinearVelocity() const {
		return linearVelocity_;
	}

	inline double getAngularVelocity() const {
		return angularVelocity_;
	}

	void setVelocities(double linear, double angular);

	/**
	 * Returns a geometry_msgs::Twist message containing linear and angular velocities
	 * @return
	 */
	geometry_msgs::Twist::Ptr getTwistMessage() const;

	/**
	 * Rotates all path by specified angle (radians)
	 * @param angle
	 */
	void rotate(double angle);

private:

	nav_msgs::Path::Ptr path_;

	double linearVelocity_;
	double angularVelocity_;
	double weight_;

};

#endif /* INCLUDE_WANDERER_TRAJECTORY_H_ */
