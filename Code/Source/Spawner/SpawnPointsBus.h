/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AzCore/EBus/EBus.h>
#include <AzCore/Math/Vector3.h>
#include <AzCore/RTTI/BehaviorContext.h>
#include <Spawner/SpawnPointComponent.h>

namespace ROS2
{
    //! TODO: doc
    class SpawnPointsRequests : public AZ::EBusTraits
    {
        public:
        AZ_RTTI(SpawnPointsRequests, "{3C42A3A1-1B8E-4800-9473-E4441315D7C8}");
        virtual ~SpawnPointsRequests() = default;

        //! todo: doc
        virtual bool IsSpawnPointSuitable(const AZ::Vector3& point, const std::vector<AZ::Vector3>& bounding_box) const = 0;

        // todo: doc
        virtual std::vector<SpawnPointInfo> GetSpawnPoints(const std::vector<AZ::Vector3>& bounding_box) const = 0;
    };

    class SpawnPointsBusTraits : public AZ::EBusTraits
    {
    public:
        //////////////////////////////////////////////////////////////////////////
        // EBusTraits overrides
        static constexpr AZ::EBusHandlerPolicy HandlerPolicy = AZ::EBusHandlerPolicy::Single;
        static constexpr AZ::EBusAddressPolicy AddressPolicy = AZ::EBusAddressPolicy::Single;
        //////////////////////////////////////////////////////////////////////////
    };

    using SpawnPointsRequestBus = AZ::EBus<SpawnPointsRequests, SpawnPointsBusTraits>;
    using SpawnPointsInterface = AZ::Interface<SpawnPointsRequests>;
} // namespace ROS2