/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AzCore/Component/EntityId.h>
#include <AzCore/std/containers/vector.h>
#include <AzCore/std/string/string.h>

namespace VehicleDynamics
{
    //! An axle has one or more wheels attached. Axle configuration is abstracted form the physical object
    class AxleConfiguration
    {
    public:
        AxleConfiguration();
        static void Reflect(AZ::ReflectContext* context);

        //! Helper functions for wheel entities. If there is only one wheel, same value is returned from both.
        //! If no wheels are set, this will return invalid EntityId (check with IsValid())
        AZ::EntityId GetLeftWheelEntityId() const;  //! Return left-most wheel of the axis.
        AZ::EntityId GetRightWheelEntityId() const; //! Return right-most wheel of the axis.

        AZStd::string m_axleTag; //! Useful to differentiate between axles, can be empty.
        AZStd::vector<AZ::EntityId> m_axleWheels; //! One or more wheels attached to this axle (typically 2), sorted left to right.

        bool m_isSteering = false;
        bool m_isDrive = false;
    };
} // namespace VehicleDynamics
