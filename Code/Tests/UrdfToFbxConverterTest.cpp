/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <URDF/UrdfToFbxConverter.h>

#include <AzTest/AzTest.h>

namespace {

class UrdfToFbxConverterTest : public ::testing::Test
{
    public:
        std::string GetUrdfWithOneLink()
        {
            std::string xmlStr =
                "<robot name=\"test_one_link\">"
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
            return xmlStr;
        }

    protected:
        ROS2::UrdfToFbxConverter converter; 
};

TEST_F(UrdfToFbxConverterTest, ConvertUrdfWithOneLink)
{
    const auto xmlStr = GetUrdfWithOneLink();
    const auto fbxStr = converter.ConvertUrdfToFbx(xmlStr);
}

} // namespace
