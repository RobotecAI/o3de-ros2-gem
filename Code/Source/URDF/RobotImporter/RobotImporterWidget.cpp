/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <QMessageBox>
#include <QPushButton>
#include <QVBoxLayout>

#include "RobotImporterWidget.h"
#include "URDF/RobotImporter/URDFPrefabMaker.h"

namespace ROS2
{
    RobotImporterWidget::RobotImporterWidget(QWidget* parent)
        : QWidget(parent)
        , m_logLabel("", this)
        , m_robotImporter(*this)
    {
        setWindowTitle(QObject::tr("Robot definition file importer"));
        QVBoxLayout* mainLayout = new QVBoxLayout(this);
        mainLayout->setSpacing(20);
        QLabel* captionLabel = new QLabel(QObject::tr("Select a robot definition (URDF) file to import"), this);
        captionLabel->setWordWrap(true);
        mainLayout->addWidget(captionLabel);
        QPushButton* selectFileButton = new QPushButton(QObject::tr("Load"), this);
        mainLayout->addWidget(selectFileButton);
        mainLayout->addWidget(&m_logLabel);
        mainLayout->addStretch();

        QObject::connect(
            selectFileButton,
            &QPushButton::clicked,
            this,
            [this]()
            {
                m_robotImporter.Import();
            });
        setLayout(mainLayout);
    }

    void RobotImporterWidget::ReportError(AZStd::string errorMessage)
    {
        QMessageBox::critical(this, QObject::tr("Error"), errorMessage.c_str());
        m_logLabel.setText(errorMessage.c_str());
        AZ_Error("RobotImporterWidget", false, errorMessage.c_str());
    }
    void RobotImporterWidget::ReportWarning(AZStd::string warningMessage)
    {
        AZ_Warning("RobotImporterWidget", false, warningMessage.c_str());
    }
    void RobotImporterWidget::ReportInfo(AZStd::string infoMessage)
    {
        m_logLabel.setText(infoMessage.c_str());
        AZ_TracePrintf("RobotImporterWidget", infoMessage.c_str());
    }
    AZStd::string RobotImporterWidget::GetURDFPath()
    {
        return GetPathWithExtension("Unified Robot Description Format (*.urdf)", QFileDialog::ExistingFiles);
    }

    AZStd::string RobotImporterWidget::GetNewPrefabPath()
    {
        return GetPathWithExtension("Prefab (*.prefab)", QFileDialog::AnyFile);
    }

    RobotImporterInputInterface::ExistingPrefabAction RobotImporterWidget::GetExistingPrefabAction()
    {
        QMessageBox msgBox;
        msgBox.setText("The prefab file already exists");
        msgBox.setInformativeText("Do you want to overwrite it or choose a new filename?");
        msgBox.setStandardButtons(QMessageBox::Save | QMessageBox::Discard | QMessageBox::Cancel);
        msgBox.setDefaultButton(QMessageBox::Save);
        msgBox.setButtonText(QMessageBox::Save, "Overwrite");
        msgBox.setButtonText(QMessageBox::Discard, "Save As..");
        auto ret = msgBox.exec();
        switch (ret)
        {
        case QMessageBox::Save:
            return ExistingPrefabAction::Overwrite;
        case QMessageBox::Discard:
            return ExistingPrefabAction::NewName;
        case QMessageBox::Cancel:
            return ExistingPrefabAction::Cancel;
        default:
            return ExistingPrefabAction::Cancel;
        }
    }

    AZStd::string RobotImporterWidget::GetPathWithExtension(AZStd::string extensionDescription, QFileDialog::FileMode mode)
    {
        QFileDialog m_importFileDialog;
        m_importFileDialog.setFileMode(mode);
        m_importFileDialog.setNameFilter(QObject::tr(extensionDescription.c_str()));
        m_importFileDialog.setViewMode(QFileDialog::Detail);

        int result = m_importFileDialog.exec();
        if (result != QDialog::DialogCode::Accepted)
        {
            return {};
        }

        return m_importFileDialog.selectedFiles().first().toStdString().c_str();
    }

} // namespace ROS2
