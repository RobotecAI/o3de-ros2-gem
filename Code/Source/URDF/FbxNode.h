/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <string>
#include <any>

namespace ROS2
{
    namespace Fbx
    {
        using Property = std::any;
        using Properties = std::vector<Property>;

        //! Represents raw string without quotation marks for Node properties purposes.
        //!
        //! String values are quoted by default when added as node properties.
        //! But in very rare cases it's necessary to add field without quotation marks.
        struct RawString
        {
            RawString(const std::string & str)
                : data(str) {}
            std::string data;
        };

        //! A node in FBX file tree structure.
        //! Each named node could contain children nodes (subnodes) and properties.
        class Node
        {
        public:
            Node(const std::string & name, const Properties & properties = {});

            std::string GetName() const;
            std::vector<Node> GetChildren() const;
            Properties GetProperties() const;
            bool HasChildren() const;
            bool HasProperties() const;

            //! Add new property to existing node
            void AddProperty(const Property & property);

            //! Add new child node to existing node
            void AddChildNode(const Node & child);
            void AddChildNode(const std::string & name, const Property & property);
            void AddChildNode(const Node && child);

            //! Convert the node to string (ASCII Fbx).
            std::string ToString(int nodeDepth = 0)  const;

        private:
            std::string m_name;
            std::vector<Node> m_children;
            Properties m_properties;
        };

    } // namespace Fbx
} // namespace ROS2