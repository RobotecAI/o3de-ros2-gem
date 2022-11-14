/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "RobotImporter/Utils/RobotImporterUtils.h"
#include <AzCore/UnitTest/TestTypes.h>
#include <AzCore/std/string/string.h>
#include <AzTest/AzTest.h>
#include <RobotImporter/URDF/UrdfParser.h>

namespace UnitTest
{

    class UrdfParserTest : public AllocatorsTestFixture
    {
    public:
        AZStd::string GetUrdfWithOneLink()
        {
            return "<robot name=\"test_one_link\">"
                   "  <material name=\"black\">\n"
                   "    <color rgba=\"0 0 0 1\"/>\n"
                   "  </material>"
                   "  <link name=\"link1\">"
                   "    <inertial>"
                   "      <mass value=\"1.0\"/>"
                   "      <inertia ixx=\"1.0\" iyy=\"1.0\" izz=\"1.0\" ixy=\"0\" ixz=\"0\" iyz=\"0\"/>"
                   "    </inertial>"
                   "    <visual>"
                   "      <geometry>"
                   "        <box size=\"1.0 2.0 1.0\"/>"
                   "      </geometry>"
                   "      <material name=\"black\"/>"
                   "    </visual>"
                   "    <collision>"
                   "      <geometry>"
                   "        <box size=\"1.0 2.0 1.0\"/>"
                   "      </geometry>"
                   "    </collision>"
                   "  </link>"
                   "</robot>";
        }

        AZStd::string GetUrdfWithTwoLinksAndJoint()
        {
            return "<robot name=\"test_two_links_one_joint\">  "
                   "  <material name=\"black\">\n"
                   "    <color rgba=\"0 0 0 1\"/>\n"
                   "  </material>"
                   "  <link name=\"link1\">"
                   "    <inertial>"
                   "      <mass value=\"1.0\"/>"
                   "      <inertia ixx=\"1.0\" iyy=\"1.0\" izz=\"1.0\" ixy=\"0\" ixz=\"0\" iyz=\"0\"/>"
                   "    </inertial>"
                   "    <visual>"
                   "      <geometry>"
                   "        <box size=\"1.0 2.0 1.0\"/>"
                   "      </geometry>"
                   "      <material name=\"black\"/>"
                   "    </visual>"
                   "  </link>"
                   "  <link name=\"link2\">"
                   "    <inertial>"
                   "      <mass value=\"1.0\"/>"
                   "      <inertia ixx=\"1.0\" iyy=\"1.0\" izz=\"1.0\" ixy=\"0\" ixz=\"0\" iyz=\"0\"/>"
                   "    </inertial>"
                   "    <visual>"
                   "      <geometry>"
                   "        <box size=\"1.0 1.0 1.0\"/>"
                   "      </geometry>"
                   "      <material name=\"black\"/>"
                   "    </visual>"
                   "  </link>"
                   "  <joint name=\"joint12\" type=\"fixed\">"
                   "    <parent link=\"link1\"/>"
                   "    <child link=\"link2\"/>"
                   "    <origin rpy=\"0 0 0\" xyz=\"1.0 0.5 0.0\"/>"
                   "    <dynamics damping=\"10.0\" friction=\"5.0\"/>"
                   "    <limit lower=\"10.0\" upper=\"20.0\" effort=\"90.0\" velocity=\"10.0\"/>"
                   "  </joint>"
                   "</robot>";
        }
        AZStd::string GetURDFWithWheel(
            const AZStd::string& wheel_name,
            const AZStd::string& wheel_joint_type,
            bool wheel_has_visual = true,
            bool wheel_has_collider = true)
        {
            // clang-format off
            return "<robot name=\"wheel_test\">\n"
                   "    <link name=\"base_link\">\n"
                   "        <inertial>\n"
                   "            <origin xyz=\"0. 0. 0.\"/>\n"
                   "            <mass value=\"1.\"/>\n"
                   "            <inertia ixx=\"1.\" ixy=\"0.\" ixz=\"0.\" iyy=\"1.\" iyz=\"0.\" izz=\"1.\"/>\n"
                   "        </inertial>\n"
                   "    </link>\n"
                   "    <link name=\""+wheel_name+"\">\n"
                   "        <inertial>\n"
                   "            <origin xyz=\"0. 0. 0.\"/>\n"
                   "            <mass value=\"1.\"/>\n"
                   "            <inertia ixx=\"1.\" ixy=\"0.\" ixz=\"0.\" iyy=\"1.\" iyz=\"0.\" izz=\"1.\"/>\n"
                   "        </inertial>\n"
                   +(wheel_has_visual?"<visual>\n"
                   "            <origin rpy=\"0 0 0\" xyz=\"0 0 0\"/>\n"
                   "            <geometry>\n"
                   "                <box size=\"1 1 1\"/>\n"
                   "            </geometry>\n"
                   "        </visual>\n":"")+
                   +(wheel_has_collider?"<collision>\n"
                   "            <origin rpy=\"0 0 0\" xyz=\"0 0 0\"/>\n"
                   "            <geometry>\n"
                   "                <box size=\"1 1 1\"/>\n"
                   "            </geometry>\n"
                   "        </collision>\n":"")+
                   "    </link>\n"
                   "    <joint name=\"joint0\" type=\""+wheel_joint_type+"\">\n"
                   "        <parent link=\"base_link\"/>\n"
                   "        <child link=\""+wheel_name+"\"/>\n"
                   "        <axis xyz=\"0. 0. 1.\"/>\n"
                   "        <origin rpy=\"0. 0. 0.\" xyz=\"2. 0. 0.\"/>\n"
                   "    </joint>\n"
                   "</robot>";
            // clang-format on
        }
    };

    TEST_F(UrdfParserTest, ParseUrdfWithOneLink)
    {
        ROS2::UrdfParser parser;

        const auto xmlStr = GetUrdfWithOneLink();
        const auto urdf = parser.Parse(xmlStr);

        EXPECT_EQ(urdf->getName(), "test_one_link");

        std::vector<urdf::LinkSharedPtr> links;
        urdf->getLinks(links);
        EXPECT_EQ(links.size(), 1U);

        const auto link1 = urdf->getLink("link1");

        ASSERT_TRUE(link1);
        EXPECT_EQ(link1->inertial->mass, 1.0);
        EXPECT_EQ(link1->inertial->ixx, 1.0);
        EXPECT_EQ(link1->inertial->ixy, 0.0);
        EXPECT_EQ(link1->inertial->ixz, 0.0);
        EXPECT_EQ(link1->inertial->iyy, 1.0);
        EXPECT_EQ(link1->inertial->iyz, 0.0);
        EXPECT_EQ(link1->inertial->izz, 1.0);

        EXPECT_EQ(link1->visual->material->name, "black");
        EXPECT_EQ(link1->visual->geometry->type, 1);
        const auto visualBox = std::dynamic_pointer_cast<urdf::Box>(link1->visual->geometry);
        EXPECT_EQ(visualBox->dim.x, 1.0);
        EXPECT_EQ(visualBox->dim.y, 2.0);
        EXPECT_EQ(visualBox->dim.z, 1.0);

        EXPECT_EQ(link1->collision->geometry->type, 1);
        const auto collisionBox = std::dynamic_pointer_cast<urdf::Box>(link1->visual->geometry);
        EXPECT_EQ(collisionBox->dim.x, 1.0);
        EXPECT_EQ(collisionBox->dim.y, 2.0);
        EXPECT_EQ(collisionBox->dim.z, 1.0);
    }

    TEST_F(UrdfParserTest, ParseUrdfWithTwoLinksAndJoint)
    {
        ROS2::UrdfParser parser;

        const auto xmlStr = GetUrdfWithTwoLinksAndJoint();
        const auto urdf = parser.Parse(xmlStr);

        EXPECT_EQ(urdf->getName(), "test_two_links_one_joint");

        std::vector<urdf::LinkSharedPtr> links;
        urdf->getLinks(links);
        EXPECT_EQ(links.size(), 2U);

        const auto link1 = urdf->getLink("link1");
        ASSERT_TRUE(link1);

        const auto link2 = urdf->getLink("link2");
        ASSERT_TRUE(link2);

        const auto joint12 = urdf->getJoint("joint12");
        ASSERT_TRUE(joint12);

        EXPECT_EQ(joint12->parent_link_name, "link1");
        EXPECT_EQ(joint12->child_link_name, "link2");

        EXPECT_EQ(joint12->parent_to_joint_origin_transform.position.x, 1.0);
        EXPECT_EQ(joint12->parent_to_joint_origin_transform.position.y, 0.5);
        EXPECT_EQ(joint12->parent_to_joint_origin_transform.position.z, 0.0);

        double roll, pitch, yaw;
        joint12->parent_to_joint_origin_transform.rotation.getRPY(roll, pitch, yaw);
        EXPECT_DOUBLE_EQ(roll, 0.0);
        EXPECT_DOUBLE_EQ(pitch, 0.0);
        EXPECT_DOUBLE_EQ(yaw, 0.0);

        EXPECT_EQ(joint12->dynamics->damping, 10.0);
        EXPECT_EQ(joint12->dynamics->friction, 5.0);

        EXPECT_EQ(joint12->limits->lower, 10.0);
        EXPECT_EQ(joint12->limits->upper, 20.0);
        EXPECT_EQ(joint12->limits->effort, 90.0);
        EXPECT_EQ(joint12->limits->velocity, 10.0);
    }

    TEST_F(UrdfParserTest, WheelHeuristicNameValid)
    {
        ROS2::UrdfParser parser;
        const AZStd::string wheel_name("wheel_left_link");
        const auto xmlStr = GetURDFWithWheel(wheel_name, "continuous");
        const auto urdf = parser.Parse(xmlStr);
        auto wheel_candidate = urdf->getLink(wheel_name.c_str());
        ASSERT_TRUE(wheel_candidate);
        EXPECT_EQ(ROS2::Utils::IsWheelURDFHeuristics(wheel_candidate), true);
    }

    TEST_F(UrdfParserTest, WheelHeuristicNameNotValid1)
    {
        ROS2::UrdfParser parser;
        const AZStd::string wheel_name("wheel_left_joint");
        const auto xmlStr = GetURDFWithWheel(wheel_name, "continuous");
        const auto urdf = parser.Parse(xmlStr);
        auto wheel_candidate = urdf->getLink(wheel_name.c_str());
        ASSERT_TRUE(wheel_candidate);
        EXPECT_EQ(ROS2::Utils::IsWheelURDFHeuristics(wheel_candidate), false);
    }

    TEST_F(UrdfParserTest, WheelHeuristicJointNotValid)
    {
        ROS2::UrdfParser parser;
        const AZStd::string wheel_name("wheel_left_link");
        const auto xmlStr = GetURDFWithWheel(wheel_name, "fixed");
        const auto urdf = parser.Parse(xmlStr);
        auto wheel_candidate = urdf->getLink(wheel_name.c_str());
        ASSERT_TRUE(wheel_candidate);
        EXPECT_EQ(ROS2::Utils::IsWheelURDFHeuristics(wheel_candidate), false);
    }

    TEST_F(UrdfParserTest, WheelHeuristicJointVisualNotValid)
    {
        ROS2::UrdfParser parser;
        const AZStd::string wheel_name("wheel_left_link");
        const auto xmlStr = GetURDFWithWheel(wheel_name, "continuous", false, true);
        const auto urdf = parser.Parse(xmlStr);
        auto wheel_candidate = urdf->getLink(wheel_name.c_str());
        ASSERT_TRUE(wheel_candidate);
        EXPECT_EQ(ROS2::Utils::IsWheelURDFHeuristics(wheel_candidate), false);
    }

    TEST_F(UrdfParserTest, WheelHeuristicJointColliderNotValid)
    {
        ROS2::UrdfParser parser;
        const AZStd::string wheel_name("wheel_left_link");
        const auto xmlStr = GetURDFWithWheel(wheel_name, "continuous", true, false);
        const auto urdf = parser.Parse(xmlStr);
        auto wheel_candidate = urdf->getLink(wheel_name.c_str());
        ASSERT_TRUE(wheel_candidate);
        EXPECT_EQ(ROS2::Utils::IsWheelURDFHeuristics(wheel_candidate), false);
    }

} // namespace UnitTest
