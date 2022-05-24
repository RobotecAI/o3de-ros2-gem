/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <any>

#include <AzCore/std/containers/vector.h>
#include <AzCore/std/string/string.h>
#include <AzCore/Memory/SystemAllocator.h>

#include "UniqueIdGenerator.h"

namespace ROS2
{
    namespace Fbx
    {
        using Property = std::any;
        using Properties = AZStd::vector<Property>;
        using Nodes = AZStd::vector<class Node>;

        //! Represents raw string without quotation marks for Node properties purposes.
        //!
        //! String values are quoted by default when added as node properties.
        //! But in very rare cases it's necessary to add field without quotation marks.
        struct RawString
        {
            RawString(const AZStd::string & str)
                : data(str) {}
            AZStd::string data;
        };

        //! A node in FBX file tree structure.
        //! Each named node could contain children nodes (subnodes) and properties.
        class Node
        {
        public:
            AZ_CLASS_ALLOCATOR(Node, AZ::SystemAllocator, 0);

            Node(const AZStd::string & name,
                const Properties & properties = {}, const Nodes & children = {});

            AZStd::string GetName() const;
            Nodes GetChildren() const;
            Properties GetProperties() const;
            bool HasChildren() const;
            bool HasProperties() const;

            //! Add new property to existing node
            void AddProperty(const Property & property);

            //! Add new child node to existing node
            void AddChild(const Node & child);
            void AddChild(const AZStd::string & name, const Property & property);
            void AddChild(const Node && child);

            //! Convert the node to string (ASCII Fbx).
            AZStd::string ToString(int nodeDepth = 0)  const;

        private:
            AZStd::string m_name;
            Nodes m_children;
            Properties m_properties;
        };

        //! A node with unique id
        struct NodeWithId
        {
            NodeWithId(Id id, const Node & node)
                : id(id), node(node) {}

            NodeWithId(Id id, const Node && node)
                : id(id), node(std::move(node)) {}

            Id id;
            Node node;
        };

    } // namespace Fbx
} // namespace ROS2