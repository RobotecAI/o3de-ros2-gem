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

        //! Convert Transform from the map coordinate system.
        //! @param transform - Transform to be converted.
        //! @return Copy of a Transform in the world coordinate system.
        virtual AZ::Transform ConvertFromMapCoordinateSystem(AZ::Transform transform) = 0;

        //! Convert the position vector in WorldTM to longitude, latitude, and altitude in the
        //! geographic coordinate system relative to the map frame.
        //! @param worldPosition - position vector to be converted.
        //! @return Vector where x: longitude, y: latitude, z: altitude.
        virtual AZ::Vector3 WorldPositionToLatLon(const AZ::Vector3 &worldPosition) = 0;

        //! Convert the geographic position relative to the map frame to the global o3de coordinate system.
        //! @param latlon - geographic position in degrees.
        //! @return Vector where x: x position, y: y position, z: z position in o3de WorldTM.
        virtual AZ::Vector3 LatLonToWorldPosition(const AZ::Vector3 &latlon) = 0;


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
