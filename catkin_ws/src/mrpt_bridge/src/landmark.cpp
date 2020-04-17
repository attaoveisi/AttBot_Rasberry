/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

/*
 * File: landmark.cpp
 * Author: Vladislav Tananaev
 *
 */

#include <geometry_msgs/Pose.h>
#include "mrpt_bridge/time.h"
#include "mrpt_bridge/pose.h"
#include <mrpt_msgs/ObservationRangeBearing.h>
#include "tf/transform_datatypes.h"
#include "tf/LinearMath/Matrix3x3.h"
#include "mrpt_bridge/landmark.h"

#include <mrpt/version.h>
#include <mrpt/obs/CObservationBearingRange.h>
using namespace mrpt::obs;

namespace mrpt_bridge
{
bool convert(
	const mrpt_msgs::ObservationRangeBearing& _msg,
	const mrpt::poses::CPose3D& _pose, CObservationBearingRange& _obj)

{
	// mrpt_bridge::convert(_msg.header.stamp, _obj.timestamp);
	mrpt::poses::CPose3D cpose_obj;

	//_obj.stdError = _msg.sensor_std_range;
	//_obj.sensorLabel = _msg.header.frame_id;
	_obj.maxSensorDistance = _msg.max_sensor_distance;
	_obj.minSensorDistance = _msg.min_sensor_distance;
	_obj.sensor_std_yaw = _msg.sensor_std_yaw;
	_obj.sensor_std_pitch = _msg.sensor_std_pitch;
	_obj.sensor_std_range = _msg.sensor_std_range;

	if (_pose.empty())
	{
		convert(_msg.sensor_pose_on_robot, cpose_obj);
		_obj.setSensorPose(cpose_obj);
	}
	else
	{
		_obj.setSensorPose(_pose);
	}

	ASSERT_(_msg.sensed_data.size() >= 1);
	const size_t N = _msg.sensed_data.size();

	_obj.sensedData.resize(N);

	for (std::size_t i_mrpt = 0; i_mrpt < N; i_mrpt++)
	{
		_obj.sensedData[i_mrpt].range = _msg.sensed_data[i_mrpt].range;
		_obj.sensedData[i_mrpt].landmarkID = _msg.sensed_data[i_mrpt].id;
		_obj.sensedData[i_mrpt].yaw = _msg.sensed_data[i_mrpt].yaw;
		_obj.sensedData[i_mrpt].pitch = _msg.sensed_data[i_mrpt].pitch;
	}
	return true;
}

bool convert(
	const CObservationBearingRange& _obj,
	mrpt_msgs::ObservationRangeBearing& _msg)
{
	mrpt::poses::CPose3D cpose_obj;

	// mrpt_bridge::convert(_obj.timestamp, _msg.header.stamp);
	_obj.getSensorPose(cpose_obj);
	convert(cpose_obj, _msg.sensor_pose_on_robot);

	_msg.max_sensor_distance = _obj.maxSensorDistance;
	_msg.min_sensor_distance = _obj.minSensorDistance;
	_msg.sensor_std_yaw = _obj.sensor_std_yaw;
	_msg.sensor_std_pitch = _obj.sensor_std_pitch;
	_msg.sensor_std_range = _obj.sensor_std_range;

	ASSERT_(_obj.sensedData.size() >= 1);
	const size_t N = _obj.sensedData.size();

	_msg.sensed_data.resize(N);

	for (std::size_t i_msg = 0; i_msg < N; i_msg++)
	{
		_msg.sensed_data[i_msg].range = _obj.sensedData[i_msg].range;
		_msg.sensed_data[i_msg].id = _obj.sensedData[i_msg].landmarkID;
		_msg.sensed_data[i_msg].yaw = _obj.sensedData[i_msg].yaw;
		_msg.sensed_data[i_msg].pitch = _obj.sensedData[i_msg].pitch;
	}
	return true;
}

bool convert(
	const CObservationBearingRange& _obj,
	mrpt_msgs::ObservationRangeBearing& _msg, geometry_msgs::Pose& _pose)
{
	convert(_obj, _msg);
	mrpt::poses::CPose3D pose;
	_obj.getSensorPose(pose);
	convert(pose, _pose);
	return true;
}
}  // namespace mrpt_bridge
