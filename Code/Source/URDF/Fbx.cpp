/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "Fbx.h"

#include <fstream>
#include <string>

#include <AzCore/Debug/Trace.h>

namespace ROS2
{
    namespace Fbx
    {
        using FileType = Fbx::FileType;

        void Fbx::SaveToFile(const std::string & filePath, FileType type)
        {
            // TODO: add support for binary files
            if (type != FileType::Text)
            {
                std::runtime_error(std::string(__func__) + ": Only text file type is supported!");
            }

            std::ofstream file(filePath);
            if (!file.is_open())
            {
                std::runtime_error(std::string(__func__) + ": Unable to open file: " + filePath);
            }

            file << GetFbxString();
            AZ_Printf("Fbx", "Data structure saved to file: %s", filePath.c_str());
        }

        std::string Fbx::GetFbxString()
        {
            if (!m_nodesUpdated)
            {
                GenerateFbxStructure();
            }

            std::string data;
            // Traverse tree in depth-first order and generate string
            for (const auto & n : m_basicNodes)
            {
                data += n.ToString(0);
            }

            return data;
        }

        void Fbx::Reset()
        {
            m_basicNodes.clear();
            m_nodesUpdated = false;
            m_connections.clear();
        }

        void Fbx::GenerateFbxStructure()
        {
            m_basicNodes.clear();

            m_basicNodes.push_back(GetFbxHeaderExtension());
            m_basicNodes.push_back(GetGlobalSettings());
            m_basicNodes.push_back(GetDocuments());
            m_basicNodes.push_back(Node("References"));
            m_basicNodes.push_back(GetDefinitions());
            m_basicNodes.push_back(GetObjects());
            m_basicNodes.push_back(GetConnections());
            m_basicNodes.push_back(Node("Takes"));

            m_nodesUpdated = true;
        }

        Node Fbx::GetFbxHeaderExtension() const
        {
            Node fbxHeader("FBXHeaderExtension");
            fbxHeader.AddChildNode("FBXHeaderVersion", 1003);
            fbxHeader.AddChildNode("FBXVersion", 7500);
            fbxHeader.AddChildNode(GetTimeStamp());
            fbxHeader.AddChildNode("Creator", "O3DE ROS2 Gem");
            fbxHeader.AddChildNode(GetSceneInfo());

            return fbxHeader;
        }

        Node Fbx::GetGlobalSettings() const
        {
            Node globalSettings("GlobalSettings");
            globalSettings.AddChildNode("Version", 1000);

            Node properties("Properties70");
            properties.AddChildNode(Node("P", {"UpAxis", "int", "Integer", "", 1}));
            properties.AddChildNode(Node("P", {"UpAxisSign", "int", "Integer", "", 1}));
            properties.AddChildNode(Node("P", {"FrontAxis", "int", "Integer", "", 2}));
            properties.AddChildNode(Node("P", {"FrontAxisSign", "int", "Integer", "", 1}));
            properties.AddChildNode(Node("P", {"CoordAxis", "int", "Integer", "", 0}));
            properties.AddChildNode(Node("P", {"CoordAxisSign", "int", "Integer", "", 1}));
            properties.AddChildNode(Node("P", {"OriginalUpAxis", "int", "Integer", "", 1}));
            properties.AddChildNode(Node("P", {"OriginalUpAxisSign", "int", "Integer", "", 1}));
            properties.AddChildNode(Node("P", {"UnitScaleFactor", "double", "Number", "", 1}));
            properties.AddChildNode(Node("P", {"OriginalUnitScaleFactor", "double", "Number", "", 1}));
            properties.AddChildNode(Node("P", {"AmbientColor", "ColorRGB", "Color", "", 0, 0, 0}));
            properties.AddChildNode(Node("P", {"DefaultCamera", "KString", "", "", "Producer Perspective"}));
            properties.AddChildNode(Node("P", {"TimeMode", "enum", "", "", 11}));
            properties.AddChildNode(Node("P", {"TimeProtocol", "enum", "", "", 2}));
            properties.AddChildNode(Node("P", {"SnapOnFrameMode", "enum", "", "", 0}));
            properties.AddChildNode(Node("P", {"TimeSpanStart", "KTime", "Time", "", 1924423250}));
            properties.AddChildNode(Node("P", {"TimeSpanStop", "KTime", "Time", "", 38488465000}));
            properties.AddChildNode(Node("P", {"CustomFrameRate", "double", "Number", "", -1}));
            properties.AddChildNode(Node("P", {"TimeMarker", "Compound", "", ""}));
            properties.AddChildNode(Node("P", {"CurrentTimeMarker", "int", "Integer", "", -1}));

            globalSettings.AddChildNode(std::move(properties));

            return globalSettings;
        }

        Node Fbx::GetDocuments() const
        {
            Node documents("Documents");
            documents.AddChildNode("Count", 1);

            Node document("Document", {"", "Scene"});

            Node properties("Properties70");
            properties.AddChildNode(Node("P", {"SourceObject", "object", "", ""}));
            properties.AddChildNode(Node("P", {"ActiveAnimStackName", "KString", "", "Take 001"}));

            document.AddChildNode(std::move(properties));
            document.AddChildNode("RootNode", 0);
            return documents;
        }

        Node Fbx::GetDefinitions() const
        {
            Node definitions("Definitions");
            definitions.AddChildNode("Version", 100);
            definitions.AddChildNode("Count", 3);

            Node globalSettings("ObjectType", {"GlobalSettings"});
            globalSettings.AddChildNode("Count", 1);
            definitions.AddChildNode(std::move(globalSettings));

            Node model("ObjectType", {"Model"});
            model.AddChildNode("Count", 1);
            definitions.AddChildNode(std::move(model));

            Node geometry("ObjectType", {"Geometry"});
            geometry.AddChildNode("Count", 1);
            definitions.AddChildNode(std::move(geometry));

            Node material("ObjectType", {"Material"});
            material.AddChildNode("Count", 1);
            definitions.AddChildNode(std::move(material));

            return definitions;
        }

        Node Fbx::GetObjects()
        {
            Node objects("Objects");

            // TODO: generate proper IDs
            int modelId = 1;
            int geometryId = 2;
            int materialId = 3;

            { // Example cube model
                Node model("Model", {modelId, "Model::cube", "Mesh"});
                model.AddChildNode("Version", 232);
                model.AddChildNode("Culling", "CullingOff");

                Node properties("Properties70");
                properties.AddChildNode(Node("P", {"RotationActive", "bool", "", "", 1}));
                properties.AddChildNode(Node("P", {"InheritType", "enum", "", "", 1}));
                properties.AddChildNode(Node("P", {"ScalingMax", "Vector3D", "Vector", "", 0, 0, 0}));
                properties.AddChildNode(Node("P", {"DefaultAttributeIndex", "int", "Integer", "", 0}));
                properties.AddChildNode(Node("P", {"Lcl Scaling", "Lcl Scaling", "", "A", 100, 100, 100}));
                properties.AddChildNode(Node("P", {"currentUVSet", "KString", "", "U", "map1"}));
                model.AddChildNode(std::move(properties));

                objects.AddChildNode(std::move(model));
            }

            { // Example cube material
                Node material("Material", {materialId, "Material::cube", ""});
                material.AddChildNode("Version", 102);
                material.AddChildNode("ShadingModel", "phong");
                material.AddChildNode("Multilayer", 0);

                Node properties("Properties70");
                properties.AddChildNode(Node("P", {"AmbientColor", "Color", "", "A", 0, 0, 0}));
                properties.AddChildNode(Node("P", {"DiffuseColor", "Color", "", "A", 1, 1, 1}));
                properties.AddChildNode(Node("P", {"DiffuseFactor", "Number", "", "A", 0.9}));
                properties.AddChildNode(Node("P", {"TransparencyFactor", "Number", "", "A", 1}));
                properties.AddChildNode(Node("P", {"SpecularColor", "Color", "", "A", 0.5, 0.5, 0.5}));
                properties.AddChildNode(Node("P", {"ReflectionFactor", "Number", "", "A", 0.5}));
                properties.AddChildNode(Node("P", {"Emissive", "Vector3D", "Vector", "", 0, 0, 0}));
                properties.AddChildNode(Node("P", {"Ambient", "Vector3D", "Vector", "", 0, 0, 0}));
                properties.AddChildNode(Node("P", {"Diffuse", "Vector3D", "Vector", "", 0.9, 0.9, 0.9}));
                properties.AddChildNode(Node("P", {"Specular", "Vector3D", "Vector", "", 0.5, 0.5, 0.5}));
                properties.AddChildNode(Node("P", {"Shininess", "double", "Number", "", 20}));
                properties.AddChildNode(Node("P", {"Opacity", "double", "Number", "", 1}));
                properties.AddChildNode(Node("P", {"Reflectivity", "double", "Number", "", 0}));
                material.AddChildNode(std::move(properties));

                objects.AddChildNode(std::move(material));
            }

            // Add model, syntax is as below
            // Model: "name", "Mesh" {
            //      Vertices: [...]
            //      PolygonVertexIndex: [...]
            //      LayerElementNormal: { }
            //      LayerElementUV: { }
            // }
            double cubeSize = 0.5;
            const auto geometry = CreateGeometryCube(geometryId, cubeSize);
            objects.AddChildNode(std::move(geometry));

            // TODO: change to auto generation
            auto c = Connection(0, modelId, "OO");
            m_connections.push_back(c);
            m_connections.push_back(Connection(modelId, geometryId, "OO"));
            m_connections.push_back(Connection(modelId, materialId, "OO"));

            return objects;
        }

        Node Fbx::CreateGeometryCube(int id, double size) const
        {
            // Example cube geometry
            Node geometry("Geometry", {id, "Geometry::cube", "Mesh"});
            geometry.AddChildNode("GeometryVersion", 102);
            geometry.AddChildNode(
                // How vertices are defined
                // Vertex v1: v1_x, v1_y, v1_z
                // For more vertices the definitions looks as follows:
                // Vertices: v1_x, v1_y, v1_z, v2_x, v2_y, v2_z, ..., vn_x, vn_y, vn_z
                // More details: https://banexdevblog.wordpress.com/2014/06/23/a-quick-tutorial-about-the-fbx-ascii-format/
                Node("Vertices", {-size,-size,size,
                                size,-size,size,
                                -size,size,size,
                                size,size,size,
                                -size,size,-size,
                                size,size,-size,
                                -size,-size,-size,
                                size,-size,-size}));

            // Indices of four-sided polygons (quads)
            geometry.AddChildNode(
                Node("PolygonVertexIndex", {0,1,3,-3,
                                            2,3,5,-5,
                                            4,5,7,-7,
                                            6,7,1,-1,
                                            1,7,5,-4,
                                            6,0,2,-5}));

            // Edges
            geometry.AddChildNode(
                Node("Edges", {0,2,6,10,3,1,7,5,11,9,15,13}));

            // Normals
            auto layerElementNormal = Node("LayerElementNormal", {0});
            layerElementNormal.AddChildNode("Version", 102);
            layerElementNormal.AddChildNode("Name", "");
            layerElementNormal.AddChildNode("MappingInformationType", "ByPolygonVertex");
            layerElementNormal.AddChildNode("ReferenceInformationType", "Direct");
            layerElementNormal.AddChildNode(
                Node("Normals", {0,0,1,0,0,1,0,0,1,0,0,1,0,1,0,0,1,0,0,1,0,0,1,0,0,0,-1,0,0,-1,0,0,-1,0,0,-1,0,
                                -1,0,0,-1,0,0,-1,0,0,-1,0,1,0,0,1,0,0,1,0,0,1,0,0,-1,0,0,-1,0,0,-1,0,0,-1,0,0}));
            layerElementNormal.AddChildNode(
                Node("NormalsW", {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1}));
            geometry.AddChildNode(layerElementNormal);

            // UV
            auto layerElementUV = Node("LayerElementUV", {0});
            layerElementUV.AddChildNode("Version", 101);
            layerElementUV.AddChildNode("Name", "map1");
            layerElementUV.AddChildNode("MappingInformationType", "ByPolygonVertex");
            layerElementUV.AddChildNode("ReferenceInformationType", "IndexToDirect");
            layerElementUV.AddChildNode(
                Node("UV", {0.375,0,0.625,0,0.375,0.25,0.625,0.25,0.375,0.5,0.625,0.5,0.375,0.75,
                            0.625,0.75,0.375,1,0.625,1,0.875,0,0.875,0.25,0.125,0,0.125,0.25}));
            layerElementUV.AddChildNode(
                Node("UVIndex", {0,1,3,2,2,3,5,4,4,5,7,6,6,7,9,8,1,10,11,3,12,0,2,13}));

            geometry.AddChildNode(layerElementUV);

            return geometry;
        }

        Node Fbx::GetConnections() const
        {
            Node connections("Connections");
            for (const auto & c : m_connections)
            {
                connections.AddChildNode(Node("C", {c.type, c.childId, c.parentId}));
            }

            return connections;
        }

        Node Fbx::GetTimeStamp() const
        {
            // TODO: get proper time stamp
            Node timeStamp("CreationTimeStamp");
            timeStamp.AddChildNode("Version", 1000);
            timeStamp.AddChildNode("Year", 2022);
            timeStamp.AddChildNode("Month", 01);
            timeStamp.AddChildNode("Day", 01);
            timeStamp.AddChildNode("Hour", 0);
            timeStamp.AddChildNode("Minute", 0);
            timeStamp.AddChildNode("Second", 0);
            timeStamp.AddChildNode("Millisecond", 0);

            return timeStamp;
        }

        Node Fbx::GetSceneInfo() const
        {
            Node sceneInfo("SceneInfo");
            sceneInfo.AddProperty("SceneInfo::GlobalInfo");
            sceneInfo.AddProperty("UserData");
            sceneInfo.AddChildNode("Type", "UserData");
            sceneInfo.AddChildNode("Version", 100);
            sceneInfo.AddChildNode(GetMetaData());

            Node properties("Properties70");
            properties.AddChildNode(
                Node("P", {"DocumentUrl", "KString", "Url", "", "/dummy_path.fbx"}));
            properties.AddChildNode(
                Node("P", {"SrcDocumentUrl", "KString", "Url", "", "/dummy_path.fbx"}));
            properties.AddChildNode(
                Node("P", {"Original", "Compound", "", ""}));
            properties.AddChildNode(
                Node("P", {"Original|ApplicationVendor", "KString", "", "", "O3DE"}));
            properties.AddChildNode(
                Node("P", {"Original|ApplicationName", "KString", "", "", "O3DE"}));
            properties.AddChildNode(
                Node("P", {"Original|ApplicationVersion", "KString", "", "", "2022"}));
            properties.AddChildNode(
                Node("P", {"Original|DateTime_GMT", "DateTime", "", "", "01/01/2022 00:00:00.000"}));
            properties.AddChildNode(
                Node("P", {"Original|FileName", "KString", "", "", "/dummy_path.fbx"}));
            properties.AddChildNode(
                Node("P", {"LastSaved", "Compound", "", ""}));
            properties.AddChildNode(
                Node("P", {"LastSaved|ApplicationVendor", "KString", "", "", "O3DE"}));
            properties.AddChildNode(
                Node("P", {"LastSaved|ApplicationName", "KString", "", "", "O3DE"}));
            properties.AddChildNode(
                Node("P", {"LastSaved|ApplicationVersion", "KString", "", "", "2022"}));
            properties.AddChildNode(
                Node("P", {"LastSaved|DateTime_GMT", "DateTime", "", "", "01/01/2022 00:00:00.000"}));
            properties.AddChildNode(
                Node("P", {"Original|ApplicationActiveProject", "KString", "", "", "/dummy_path.fbx"}));
            properties.AddChildNode(
                Node("P", {"Original|ApplicationNativeFile", "KString", "", "", "/dummy_path.fbx"}));

            sceneInfo.AddChildNode(std::move(properties));

            return sceneInfo;
        }

        Node Fbx::GetMetaData() const
        {
            Node metaData("MetaData");
            metaData.AddChildNode("Version", 100);
            metaData.AddChildNode("Title", "");
            metaData.AddChildNode("Subject", "");
            metaData.AddChildNode("Author", "");
            metaData.AddChildNode("Keywords", "");
            metaData.AddChildNode("Revision", "");
            metaData.AddChildNode("Comment", "");

            return metaData;
        }

    } // namespace Fbx
} // namespace ROS2