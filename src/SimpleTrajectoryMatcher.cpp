/**
 * Filename: SimpleTrajectoryMatcher.cpp
 *   Author: Igor Makhtes
 *     Date: Nov 29, 2014
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

#include <wanderer/SimpleTrajectoryMatcher.h>

SimpleTrajectoryMatcher::SimpleTrajectoryMatcher() {
}

SimpleTrajectoryMatcher::~SimpleTrajectoryMatcher() {
}

TrajectoryMatch::Ptr SimpleTrajectoryMatcher::match(CostMap& costMap,
		Trajectory::Ptr trajectory) const {

	const nav_msgs::Path::Ptr path = trajectory->getPath();

	/**
	 * Score ranges from -1 (Unknown) to 100 (definitely occupied)
	 */
	const int pointScoreRange = 101;

	/**
	 * Maximum possible trajectory weight
	 */
	const double maxTrajectoryWeight = 1;

	/**
	 * Maximum score, used to normalize the value to range 0..1
	 */
	double maxPathScore = path->poses.size() * pointScoreRange * maxTrajectoryWeight;

	bool fatalPath = false;
	double scoreSum = 0;

	for (int i = 0; i < path->poses.size(); ++i) {
		const geometry_msgs::Pose& pose = path->poses[i].pose;
		double pointValue = 1 + costMap.getCellValue(pose);

		/**
		 * Fatal path check
		 */
		if (pointValue == 101) {
			/**
			 * The path is definitely blocked, mark it as fatal
			 */
			fatalPath = true;
		}

		scoreSum += pointValue;
	}

	/**
	 * Empty path
	 */
	if (maxPathScore == 0)
		return TrajectoryMatch::Ptr(new TrajectoryMatch(trajectory, 0));

	/**
	 * Normalize and invert
	 */
	double finalScore = (1 - scoreSum / maxPathScore) * trajectory->getWeight();

	/**
	 * Fatal path, adjust the score to be in range [-1,0]
	 */
	if (fatalPath)
		finalScore -= 1;

	TrajectoryMatch::Ptr trajectoryMatch(new TrajectoryMatch(trajectory, finalScore));
	return trajectoryMatch;
}
