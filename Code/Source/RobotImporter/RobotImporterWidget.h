/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#if !defined(Q_MOC_RUN)
#include "URDF/URDFPrefabMaker.h"
#include "URDF/UrdfParser.h"
#include <AzCore/Asset/AssetCommon.h>
#include <AzCore/std/containers/unordered_map.h>
#include <RobotImporter/Utils/RobotImporterUtils.h>

#include "RobotImporter/URDF/RobotImporter.h"
#include <AzToolsFramework/API/ToolsApplicationAPI.h>
#include <AzToolsFramework/Prefab/PrefabFocusInterface.h>
#include <QCheckBox>
#include <QFileDialog>
#include <QFileSystemModel>
#include <QHeaderView>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QPushButton>
#include <QTableWidget>
#include <QTextEdit>
#include <QTimer>
#include <QVBoxLayout>
#include <QWidget>
#include <QWizard>
#include <QWizardPage>
#endif

namespace ROS2
{
    class RobotImporterWidget;
    class URDFPrefabMaker;
    class FileSelectionPage : public QWizardPage
    {
        Q_OBJECT
    public:
        explicit FileSelectionPage(QWizard* parent);
        bool isComplete() const override;
        QString getFileName() const
        {
            if (m_fileExists)
            {
                return m_textEdit->text();
            }
            return "";
        }

    private:
        QFileDialog* m_fileDialog;
        QPushButton* m_button;
        QLineEdit* m_textEdit;
        void onLoadButtonPressed();
        void onFileSelected(const QString& file);
        void onTextChanged(const QString& text);
        bool m_fileExists{ false };
    };

    class CheckUrdfPage : public QWizardPage
    {
        Q_OBJECT
    public:
        explicit CheckUrdfPage(QWizard* parent);
        void ReportURDFResult(const QString& result, bool is_success);
        bool isComplete() const override;

    private:
        QTextEdit* m_log;
        QString m_fileName;
        bool m_success;
    };

    class CheckAssetPage : public QWizardPage
    {
        Q_OBJECT
    public:
        explicit CheckAssetPage(QWizard* parent);
        void ReportAsset(
            const AZStd::string& urdfPath,
            const AZStd::string& type,
            const AZStd::string& assetSourcePath,
            const AZ::Crc32& crc32,
            const AZStd::string& tooltip);
        void ClearAssetsList();

        bool isComplete() const override;

    private:
        bool m_success;
        QTableWidget* m_table;
        QTableWidgetItem* createCell(bool isOk, const AZStd::string& text);
        unsigned int m_missingCount;
        void SetTitle();
    };

    class PrefabMakerPage : public QWizardPage
    {
        Q_OBJECT
    public:
        explicit PrefabMakerPage(RobotImporterWidget* parent);
        void setProposedPrefabName(const AZStd::string prefabName);
        void reportProgress(const AZStd::string& progressForUser);
        void setSuccess(bool success);
        bool isComplete() const override;

    private:
        void onCreateButtonPressed();
        bool m_success;
        QLineEdit* m_prefabName;
        QPushButton* m_createButton;
        QTextEdit* m_log;
        RobotImporterWidget* m_parentImporterWidget;
    };

    //! Handles UI for the process of URDF importing
    class RobotImporterWidget : public QWizard
    {
        Q_OBJECT
    public:
        explicit RobotImporterWidget(QWidget* parent = nullptr);
        void CreatePrefab(AZStd::string prefabName);

    private:
        FileSelectionPage* m_fileSelectPage;
        CheckUrdfPage* m_checkUrdfPage;
        CheckAssetPage* m_assetPage;
        PrefabMakerPage* m_prefabMakerPage;
        AZStd::string m_urdfPath;
        urdf::ModelInterfaceSharedPtr m_parsedUrdf;

        /// mapping from urdf path to asset source
        AZStd::shared_ptr<AZStd::unordered_map<AZStd::string, Utils::urdf_asset>> m_urdfAssetsMapping;

        AZStd::unique_ptr<URDFPrefabMaker> m_prefabMaker;
        void ImporterTimerUpdate();
        void onCurrentIdChanged(int id);

        //! Report an information to the user.
        //! Populates the log and sets status information in the status label
        //! @param infoMessage info message to display to the user
        void ReportInfo(const AZStd::string& infoMessage);

        //! Checks if the importedPrefabFilename is the same as focused prefab name.
        //! @param importedPrefabFilename name of imported prefab
        //! @return True if names of prefabs are identical or an erorr occured during validation
        bool CheckCyclicalDependency(const AZ::IO::PathView& importedPrefabFilename);

        //! Report an error to the user.
        //! Populates the log, sets status information in the status label and shows an error popup with the message
        //! @param errorMessage error message to display to the user
        void ReportError(const AZStd::string& errorMessage);

    };
} // namespace ROS2
