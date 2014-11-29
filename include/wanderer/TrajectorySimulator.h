/**
 * Filename: TrajectorySimulator.h
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

#ifndef INCLUDE_WANDERER_TRAJECTORYSIMULATOR_H_
#define INCLUDE_WANDERER_TRAJECTORYSIMULATOR_H_


#include <tf/tf.h>

#include <wanderer/Trajectory.h>


/*
 * Simulates trajectories using linear and angular velocities
 */
class TrajectorySimulator {

public:

	TrajectorySimulator(double simulationTime, double granularity);

public:

	Trajectory::Ptr simulate(double linearVelocity, double angularVelocity) const;

	/**
	 * Sets simulation time in seconds
	 * @param simulationTime
	 */
	void setSimulationTime(double simulationTime);

	/**
	 * Gets simulation time in seconds
	 * @return
	 */
	double getSimulationTime() const;

	/**
	 * Sets simulation time step in seconds
	 * @param granularity
	 */
	void setGranularity(double granularity);

	/**
	 * Gets simulation time step in seconds
	 * @return
	 */
	double getGranularity() const;

private:

	/**
	 * Simulation time in seconds
	 */
	double simulationTime_;

	/**
	 * Simulation time step in seconds
	 */
	double granularity_;

private:

	/**
	 * Creates velocity vector using tf::Transform
	 * @param linearVelocity
	 * @param angularVelocity
	 * @param timeStep
	 * @return
	 */
	tf::Transform createVelocityTransform(double linearVelocity,
			double angularVelocity, double timeStep) const;

};

#endif /* INCLUDE_WANDERER_TRAJECTORYSIMULATOR_H_ */
