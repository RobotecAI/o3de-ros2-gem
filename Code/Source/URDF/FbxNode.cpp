/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "FbxNode.h"

#include <iomanip>
#include <fstream>
#include <string>

#include <AzCore/Debug/Trace.h>

namespace ROS2
{
    namespace Fbx
    {
        std::string AnyToString(const std::any & a)
        {
            if (a.type() == typeid(int))
            {
                return std::to_string(std::any_cast<int>(a));
            }
            else if (a.type() == typeid(unsigned))
            {
                return std::to_string(std::any_cast<unsigned>(a));
            }
            else if (a.type() == typeid(float))
            {
                return std::to_string(std::any_cast<float>(a));
            }
            else if (a.type() == typeid(double))
            {
                return std::to_string(std::any_cast<double>(a));
            }
            else if (a.type() == typeid(const char*))
            {
                return std::string("\"") + std::any_cast<const char*>(a) + std::string("\"");
            }
            else if (a.type() == typeid(std::string))
            {
                return std::string("\"") + std::any_cast<std::string>(a) + std::string("\"");
            }
            else if (a.type() == typeid(RawString))
            {
                return std::any_cast<RawString>(a).data;
            }

            AZ_Warning(__func__, false, "Unhandled type: %s", a.type().name());
            return "";
        }

        Node::Node(
            const AZStd::string & name, const Properties & properties, const Nodes & children)
            : m_name(name)
            , m_properties(properties)
            , m_children(children)
        {}

        AZStd::string Node::GetName() const
        {
            return m_name;
        }

        Nodes Node::GetChildren() const
        {
            return m_children;
        }

        Properties Node::GetProperties() const
        {
            return m_properties;
        }

        bool Node::HasChildren() const
        {
            return !m_children.empty();
        }

        bool Node::HasProperties() const
        {
            return !m_properties.empty();
        }

        void Node::AddProperty(const Property & property)
        {
            m_properties.push_back(property);
        }

        void Node::AddChild(const Node & child)
        {
            m_children.push_back(child);
        }

        void Node::AddChild(const AZStd::string & name, const Property & property)
        {
            m_children.push_back(Node(name, { property }));
        }

        void Node::AddChild(const Node && child)
        {
            m_children.push_back(child);
        }

        AZStd::string Node::ToString(int nodeDepth) const
        {
            // Calculate offset
            constexpr int spacesPerIndentLevel = 2;
            std::stringstream offsetStream;
            offsetStream << std::setfill(' ') << std::setw(nodeDepth * spacesPerIndentLevel ) << "";
            std::string offset = offsetStream.str();

            // Write name
            std::stringstream ss;
            ss << offset << m_name.data() << ": ";

            if (!HasProperties() && !HasChildren())
            {
                ss << " {\n";
                ss << offset << "}";
            }

            // Write properties
            if (HasProperties())
            {
                bool hasPrevious = false;
                for(const auto & property : m_properties)
                {
                    if(hasPrevious)
                    {
                        ss << ", ";
                    }
                    (void)property;
                    ss << AnyToString(property);
                    hasPrevious = true;
                }
            }

            // Write child nodes
            if (HasChildren())
            {
                ss << " {\n";

                if(m_children.size() > 0)
                {
                    for(auto node : m_children)
                    {
                        ss << node.ToString(nodeDepth + 1).data();
                    }
                }
                ss << offset << "}\n";
            }
            else
            {
                ss << "\n";
            }

            return ss.str().c_str();
        }

    } // namespace Fbx
} // namespace ROS2