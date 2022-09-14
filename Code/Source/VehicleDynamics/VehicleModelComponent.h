/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include "ChassisConfiguration.h"
#include "DriveModel.h"
#include "ManualControlEventHandler.h"
#include "VehicleInputControlBus.h"
#include "VehicleInputsState.h"
#include <AzCore/Component/Component.h>
#include <AzCore/Component/TickBus.h>
#include <AzCore/std/smart_ptr/unique_ptr.h>

namespace VehicleDynamics
{
    //! A central vehicle and robot) dynamics component, which can be extended with additional modules.
    class VehicleModelComponent
        : public AZ::Component
        , private VehicleInputControlRequestBus::Handler
        , private AZ::TickBus::Handler
    {
    public:
        AZ_COMPONENT(VehicleModelComponent, "{7093AE7A-9F64-4C77-8189-02C6B7802C1A}", AZ::Component);
        VehicleModelComponent() = default;

        // AZ::Component interface implementation.
        void Activate() override;
        void Deactivate() override;
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);
        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);

        // Required Reflect function.
        static void Reflect(AZ::ReflectContext* context);

    private:
        void OnTick(float deltaTime, AZ::ScriptTimePoint time) override;

        void SetTargetLinearSpeed(float speedMps) override;
        void SetTargetAcceleration(float acceleration) override;
        void SetTargetSteering(float steering) override;

        ManualControlEventHandler m_manualControlEventHandler;
        ChassisConfiguration m_chassisConfiguration;
        VehicleInputsState m_inputsState;
        AZStd::unique_ptr<DriveModel> m_driveModel;

        float m_accumulatedTimeoutSpeed = 0; // TODO - separate out into inputs timeout handler
        float m_accumulatedTimeoutSteering = 0;
        // TODO - Engine, Transmission, Lights, etc.
    };
} // namespace VehicleDynamics
