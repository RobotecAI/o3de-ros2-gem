/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <AzCore/Utils/Utils.h>
#include <QMessageBox>
#include <QPushButton>
#include <QVBoxLayout>

#include "RobotImporter/RobotImporterWidget.h"
#include "RobotImporter/RobotImporterWidgetUtils.h"
#include "RobotImporter/URDF/RobotImporter.h"

namespace ROS2
{
    namespace Internal
    {
        AZStd::string GetBaseName(AZStd::string path)
        {
            QFileInfo fileInfo(path.c_str());
            return fileInfo.baseName().toStdString().c_str();
        }

    } // namespace Internal

    RobotImporterWidget::RobotImporterWidget(QWidget* parent)
        : QWidget(parent)
        , m_statusLabel("", this)
    {
        setWindowTitle(QObject::tr("Robot definition file importer"));
        QVBoxLayout* mainLayout = new QVBoxLayout(this);
        mainLayout->setSpacing(20);
        QLabel* captionLabel = new QLabel(QObject::tr("Select a robot definition (URDF) file to import"), this);
        captionLabel->setWordWrap(true);
        mainLayout->addWidget(captionLabel);
        QPushButton* selectFileButton = new QPushButton(QObject::tr("Load"), this);
        mainLayout->addWidget(selectFileButton);
        mainLayout->addWidget(&m_statusLabel);
        mainLayout->addStretch();

        QObject::connect(
            selectFileButton,
            &QPushButton::clicked,
            this,
            [this]()
            {
                AZStd::optional<AZStd::string> urdfPath = RobotImporterWidgetUtils::QueryUserForURDFPath(this);
                if (!urdfPath)
                {
                    return;
                }

                auto prefabName = AZStd::string::format("%s.%s", Internal::GetBaseName(urdfPath.value()).c_str(), "prefab");
                AZStd::string prefabDefaultPath(AZ::IO::Path(AZ::Utils::GetProjectPath()) / "Assets" / "Importer" / prefabName);
                auto prefabPath =
                    RobotImporterWidgetUtils::ValidatePrefabPathExistenceAndQueryUserForNewIfNecessary(prefabDefaultPath, this);
                if (!prefabPath)
                {
                    ReportError("User cancelled");
                    return;
                }
                RobotImporter::Import(
                    { urdfPath.value(), prefabPath.value() },
                    [this](const AZStd::string& message)
                    {
                        ReportInfo(message);
                    },
                    [this](const AZStd::string& message)
                    {
                        ReportError(message);
                    });
            });
        setLayout(mainLayout);
    }

    void RobotImporterWidget::ReportError(const AZStd::string& errorMessage)
    {
        QMessageBox::critical(this, QObject::tr("Error"), QObject::tr(errorMessage.c_str()));
        m_statusLabel.setText(errorMessage.c_str());
        AZ_Error("RobotImporterWidget", false, errorMessage.c_str());
    }

    void RobotImporterWidget::ReportInfo(const AZStd::string& infoMessage)
    {
        m_statusLabel.setText(QObject::tr(infoMessage.c_str()));
        AZ_TracePrintf("RobotImporterWidget", infoMessage.c_str());
    }

} // namespace ROS2
