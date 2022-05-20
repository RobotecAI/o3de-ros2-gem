/*
* Copyright (c) Contributors to the Open 3D Engine Project.
* For complete copyright and license terms please see the LICENSE at the root of this distribution.
*
* SPDX-License-Identifier: Apache-2.0 OR MIT
*
*/
#pragma once


#include <AzCore/EBus/EBus.h>
#include <AzCore/Interface/Interface.h>

namespace ROS2
{
    class MapRequests
    {
    public:
        AZ_RTTI(MapRequests, "{a95c07bd-d8d6-4e24-b089-e96d34d79ebb}");
        virtual ~MapRequests() = default;

        // Put your public methods here
        virtual AZ::Vector3 LocalToLatLon(const AZ::Vector3 &local) = 0;
    };

    class MapBusTraits
            : public AZ::EBusTraits
    {
    public:
        //////////////////////////////////////////////////////////////////////////
        // EBusTraits overrides
        static constexpr AZ::EBusHandlerPolicy HandlerPolicy = AZ::EBusHandlerPolicy::Single;
        static constexpr AZ::EBusAddressPolicy AddressPolicy = AZ::EBusAddressPolicy::Single;
        //////////////////////////////////////////////////////////////////////////
    };

    using MapRequestBus = AZ::EBus<MapRequests, MapBusTraits>;

} // namespace ROS2
