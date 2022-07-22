/*
* Copyright (c) Contributors to the Open 3D Engine Project.
* For complete copyright and license terms please see the LICENSE at the root of this distribution.
*
* SPDX-License-Identifier: Apache-2.0 OR MIT
*
*/
#pragma once

#include <AzCore/std/string/string.h>
#include <AzCore/EBus/EBus.h>
#include <AzCore/Math/Vector3.h>
#include <AzCore/Math/Transform.h>
#include <AzCore/Interface/Interface.h>

namespace ROS2
{
    class MapRequests
    {
    public:
        AZ_RTTI(MapRequests, "{a95c07bd-d8d6-4e24-b089-e96d34d79ebb}");
        virtual ~MapRequests() = default;

        //! Convert Transform to the map coordinate system.
        //! @param transform - Transform to be converted.
        //! @return Copy of a Transform in the map coordinate system.
        virtual AZ::Transform ConvertToMapCoordinateSystem(AZ::Transform transform) = 0;

        //! Convert the position vector in WorldTM to longitude, latitude, and altitude in the
        //! geographic coordinate system relative to the map frame.
        //! @param local - position vector to be converted.
        //! @return Vector where x: longitude, y: latitude, z: altitude.
        virtual AZ::Vector3 LocalToLatLon(const AZ::Vector3 &local) = 0;

        //! Get available spawn positions as Transforms.
        //! @return Vector of available spawn Transforms in map frame.
        virtual AZStd::vector<AZ::Transform> GetAvailableSpawnPoints() = 0;

        //! Get map frame id.
        //! @return Map frame id.
        virtual AZStd::string GetMapFrameId() = 0;

        //! Get odometry frame id.
        //! @return Odometry frame id.
        virtual AZStd::string GetOdomFrameId() = 0;
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
    using MapRequestInterface = AZ::Interface<MapRequests>;

} // namespace ROS2
