/**
 * Filename: CostMap.cpp
 *   Author: Igor Makhtes
 *     Date: Nov 26, 2014
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

#include <wanderer/CostMap.h>

CostMap::CostMap(ICostMapDataSource* dataSource, double mapWidth,
		double mapHeight, double resolution, const string& frameId)
	: dataSource_(dataSource) {

	createOccupancyGrid(mapWidth, mapHeight, resolution, frameId);

	dataSource_->clearMapCallback =
			boost::bind(&CostMap::clearMapCallback, this);
	dataSource_->emitPointCallback =
			boost::bind(&CostMap::addPointCallback, this, _1, _2, _3, _4);
}

nav_msgs::OccupancyGrid::ConstPtr CostMap::getOccupancyGrid() const {
	return boost::const_pointer_cast<nav_msgs::OccupancyGrid const>(occupancyGrid_);
}

void CostMap::createOccupancyGrid(double mapWidth,
		double mapHeight, double resolution, const string frameId) {

	occupancyGrid_ = nav_msgs::OccupancyGrid::Ptr(
			new nav_msgs::OccupancyGrid());

	occupancyGrid_->header.frame_id = frameId;
	occupancyGrid_->info.width = mapWidth / resolution;
	occupancyGrid_->info.height = mapHeight / resolution;
	occupancyGrid_->info.resolution = resolution;
	occupancyGrid_->info.origin.position.x = -mapWidth * 0.5;
	occupancyGrid_->info.origin.position.y = -mapHeight * 0.5;
	occupancyGrid_->info.origin.orientation.w = 1;

	occupancyGrid_->data.resize(
			occupancyGrid_->info.width * occupancyGrid_->info.height);

	cvMatrix_ = cv::Mat(
			occupancyGrid_->info.height, occupancyGrid_->info.width,
			CV_8UC1, occupancyGrid_->data.data());

}

void CostMap::clearMapCallback() {
	clearMap();
}

void CostMap::clearMap() {
	memset(occupancyGrid_->data.data(), -1, occupancyGrid_->data.size());
}

void CostMap::addPointCallback(double x, double y, const string& frameId, ros::Time stamp) {
	tf::StampedTransform dataSourceToMapTransform;

	try {
		tfListener_.lookupTransform(occupancyGrid_->header.frame_id,
				frameId, ros::Time(0), dataSourceToMapTransform);
	}
	catch (tf::TransformException& exception) {
		ROS_ERROR("Failed to transform point: \n%s", exception.what());
		return;
	}

	tf::Vector3 localMapCoordinates = dataSourceToMapTransform * tf::Vector3(x, y, 0);
	cv::Point pixel = localCoordinatesToPixel(localMapCoordinates);

	if (pixel.x >= 0 && pixel.x < cvMatrix_.cols &&
			pixel.y >= 0 && pixel.y < cvMatrix_.rows)
		cvMatrix_.at<char>(pixel) = 100;
}
