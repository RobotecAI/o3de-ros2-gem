/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include "VehicleDynamics/VehicleInputControlBus.h"
#include <StartingPointInput/InputEventNotificationBus.h>

// TODO - plenty of boilerplate code, seems somewhat redundant since it would be better to be able to map inputs directly
namespace VehicleDynamics
{
    class ManualControlSingleEventHandler : private StartingPointInput::InputEventNotificationBus::Handler
    {
    public:
        using OnHeldHandlerFunction = std::function<void(float)>;
        ManualControlSingleEventHandler(AZStd::string eventName, OnHeldHandlerFunction handler)
            : m_eventName(std::move(eventName))
            , m_handler(handler)
        {
        }

        void Activate()
        {
            StartingPointInput::InputEventNotificationBus::Handler::BusConnect(
                StartingPointInput::InputEventNotificationId(m_eventName.c_str()));
        }

        void Deactivate()
        {
            StartingPointInput::InputEventNotificationBus::Handler::BusDisconnect(
                StartingPointInput::InputEventNotificationId(m_eventName.c_str()));
        }

    private:
        AZStd::string m_eventName;
        OnHeldHandlerFunction m_handler;

        void OnHeld(float value) override
        {
            m_handler(value);
        }
    };

    //! Registers to "steering" and "acceleration" input events, and translates them into vehicle inputs
    class ManualControlEventHandler
    {
    public:
        ManualControlEventHandler()
        {
            m_eventHandlers.push_back(ManualControlSingleEventHandler(
                "steering",
                [](float inputValue)
                {   // TODO handle steer
                    const float steeringLimit = 1; // Radians
                    VehicleInputControlRequestBus::Broadcast(&VehicleInputControlRequests::SetTargetSteering, inputValue * steeringLimit);
                }));

            m_eventHandlers.push_back(ManualControlSingleEventHandler(
                "accelerate",
                [](float inputValue)
                { // TODO handle speed limits.
                    const float speedLimit = 15; // Meters per second
                    VehicleInputControlRequestBus::Broadcast(&VehicleInputControlRequests::SetTargetLinearSpeed, inputValue * speedLimit);
                }));
        }

        void Activate()
        {
            for (auto& handler : m_eventHandlers)
            {
                handler.Activate();
            }
        }

        void Deactivate()
        {
            for (auto& handler : m_eventHandlers)
            {
                handler.Deactivate();
            }
        }

    private:
        AZStd::vector<ManualControlSingleEventHandler> m_eventHandlers;
    };
} // namespace VehicleDynamics