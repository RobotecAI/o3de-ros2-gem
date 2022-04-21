/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <Urdf/Fbx.h>

#include <AzTest/AzTest.h>

namespace {

class FbxTest : public ::testing::Test
{
public:
    void PrintFbxContent(const std::string & str)
    {
        std::cout << __func__ << " fbx data:"
            << "\n---------------\n"
            << str
            << "\n---------------\n";
    }

protected:
    ROS2::Fbx fbx;
};

TEST_F(FbxTest, BasicStructureGeneration)
{
    fbx.CreateFileStructure();
    const auto fbxStr = fbx.GetFbxString();
    PrintFbxContent(fbxStr);

    std::string filePath = "/home/mdrwiega/o3de/o3de-integration-test/TestData/test.fbx";
    fbx.SaveToFile(filePath);

    std::istringstream iss(fbxStr);
    std::string line;

    std::getline(iss, line);
    EXPECT_EQ(line, "FBXHeaderExtension:  {");

    std::getline(iss, line);
    EXPECT_EQ(line, "  FBXHeaderVersion: 1003");

    std::getline(iss, line);
    EXPECT_EQ(line, "  FBXVersion: 7500");

    std::getline(iss, line);
    EXPECT_EQ(line, "  CreationTimeStamp:  {");
}

} // namespace
