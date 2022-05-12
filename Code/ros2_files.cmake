# Copyright (c) Contributors to the Open 3D Engine Project.
# For complete copyright and license terms please see the LICENSE at the root of this distribution.
#
# SPDX-License-Identifier: Apache-2.0 OR MIT

set(FILES
    Source/Clock/SimulationClock.cpp
    Source/Clock/SimulationClock.h
    Source/Imu/ROS2ImuSensorComponent.cpp
    Source/Imu/ROS2ImuSensorComponent.h
    Source/QoS/QoS.cpp
    Source/QoS/QoS.h
    Source/RobotControl/RobotControl.h
    Source/RobotControl/ROS2RobotControlComponent.cpp
    Source/RobotControl/ROS2RobotControlComponent.h
    Source/RobotControl/ControlConfiguration.cpp
    Source/RobotControl/ControlConfiguration.h
    Source/RobotControl/RobotConfiguration.cpp
    Source/RobotControl/RobotConfiguration.h
    Source/RobotControl/TwistControl/TwistControl.cpp
    Source/RobotControl/TwistControl/TwistControl.h
    Source/RobotControl/TwistControl/TwistBus.h
    Source/RobotControl/TwistControl/TwistBus.cpp
    Source/ROS2ModuleInterface.h
    Source/ROS2SystemComponent.cpp
    Source/ROS2SystemComponent.h
    Source/Sensor/PublisherConfiguration.cpp
    Source/Sensor/PublisherConfiguration.h
    Source/Sensor/ROS2SensorComponent.cpp
    Source/Sensor/ROS2SensorComponent.h
    Source/Sensor/SensorConfiguration.cpp
    Source/Sensor/SensorConfiguration.h
    Source/Frame/NamespaceConfiguration.cpp
    Source/Frame/NamespaceConfiguration.h
    Source/Frame/ROS2FrameComponent.cpp
    Source/Frame/ROS2FrameComponent.h
    Source/Frame/ROS2Transform.cpp
    Source/Frame/ROS2Transform.h
    Source/Utilities/ROS2Conversions.cpp
    Source/Utilities/ROS2Conversions.h
    Source/Utilities/ROS2Names.cpp
    Source/Utilities/ROS2Names.h
)
