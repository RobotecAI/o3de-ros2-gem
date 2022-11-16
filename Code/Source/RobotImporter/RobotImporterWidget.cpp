/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <AzCore/IO/Path/Path.h>
#include <AzCore/Utils/Utils.h>
#include <QMessageBox>
#include <QVBoxLayout>

#include "RobotImporter/RobotImporterWidget.h"
#include "RobotImporter/RobotImporterWidgetUtils.h"
#include "RobotImporter/URDF/RobotImporter.h"

namespace ROS2
{
    RobotImporterWidget::RobotImporterWidget(QWidget* parent)
        : QWidget(parent)
        , m_statusText("", this)
        , m_selectFileButton(QObject::tr("Load"), this)
        , m_statusLabel(QObject::tr("Created Entities:"), this)
        , m_importerUpdateTimer(this)
        , m_robotImporter(
              [this](RobotImporter::LogLevel level, const AZStd::string& message)
              {
                  switch (level)
                  {
                  case RobotImporter::LogLevel::Info:
                      ReportInfo(message);
                      break;
                  case RobotImporter::LogLevel::Error:
                      ReportError(message);
                      break;
                  }
              })
    {
        setWindowTitle(QObject::tr("Robot definition file importer"));
        QVBoxLayout* mainLayout = new QVBoxLayout(this);
        QLabel* captionLabel = new QLabel(QObject::tr("Select a Unified Robot Description Format (URDF) file to import"), this);
        captionLabel->setWordWrap(true);
        mainLayout->addWidget(captionLabel);
        mainLayout->addWidget(&m_selectFileButton);
        mainLayout->addWidget(&m_statusLabel);
        mainLayout->addWidget(&m_statusText);
        m_statusText.setReadOnly(true);
        connect(
            &m_importerUpdateTimer,
            &QTimer::timeout,
            [this]
            {
                AZStd::string progress = m_robotImporter.GetProgress();
                m_statusText.setText(progress.c_str());
                m_robotImporter.CheckIfAssetsWereLoadedAndCreatePrefab(
                    [this]()
                    {
                        m_importerUpdateTimer.stop();
                        m_selectFileButton.setEnabled(true);
                    });
            });

        QObject::connect(
            &m_selectFileButton,
            &QPushButton::clicked,
            this,
            [this]()
            {
                AZStd::optional<AZStd::string> urdfPath = RobotImporterWidgetUtils::QueryUserForURDFPath(this);
                if (!urdfPath)
                {
                    return;
                }

                AZ::IO::Path prefabName(AZ::IO::PathView(urdfPath.value()).Filename());
                prefabName.ReplaceExtension("prefab");
                const AZ::IO::Path prefabDefaultPath(AZ::IO::Path(AZ::Utils::GetProjectPath()) / "Assets" / "Importer" / prefabName);
                auto prefabPath =
                    RobotImporterWidgetUtils::ValidatePrefabPathExistenceAndQueryUserForNewIfNecessary(prefabDefaultPath, this);
                if (!prefabPath)
                {
                    ReportError("User cancelled");
                    return;
                }

                if (IsDependencyCyclical(prefabName.Filename()))
                {
                    ReportError("Cyclical dependency detected.\nPlease choose another prefab to import or exit the focus mode.");
                    return;
                }

                m_robotImporter.ParseURDFAndStartLoadingAssets({ urdfPath.value(), prefabPath->c_str() });

                // Disable the button until the import is complete to prevent the user from clicking it again
                m_selectFileButton.setEnabled(false);
                // Check whether import is still in progress every 0.5 seconds
                m_importerUpdateTimer.start(500);
            });

        setLayout(mainLayout);
    }

    bool RobotImporterWidget::IsDependencyCyclical(const AZ::IO::PathView& importedPrefabFilename)
    {
        AzFramework::EntityContextId context_id;
        EBUS_EVENT_RESULT(context_id, AzFramework::EntityIdContextQueryBus, GetOwningContextId);

        auto focus_interface = AZ::Interface<AzToolsFramework::Prefab::PrefabFocusInterface>::Get();
        auto focus_prefab_instance = focus_interface->GetFocusedPrefabInstance(context_id);
        auto focus_prefab_filename = focus_prefab_instance.value().get().GetTemplateSourcePath().Filename();

        return focus_prefab_filename == importedPrefabFilename;
    }

    void RobotImporterWidget::ReportError(const AZStd::string& errorMessage)
    {
        QMessageBox::critical(this, QObject::tr("Error"), QObject::tr(errorMessage.c_str()));
        AZStd::string progress = m_robotImporter.GetProgress();
        m_statusText.setText(QObject::tr((progress + errorMessage).c_str()));
        AZ_Error("RobotImporterWidget", false, errorMessage.c_str());
    }

    void RobotImporterWidget::ReportInfo(const AZStd::string& infoMessage)
    {
        AZStd::string progress = m_robotImporter.GetProgress();
        m_statusText.setText(QObject::tr((progress + infoMessage).c_str()));
        AZ::Debug::Trace::Instance().Output("RobotImporterWidget", infoMessage.c_str());
    }
} // namespace ROS2
