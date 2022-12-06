/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <AzCore/IO/FileIO.h>
#include <AzCore/IO/Path/Path.h>
#include <AzCore/Utils/Utils.h>

#include "RobotImporterWidget.h"
#include "RobotImporterWidgetUtils.h"
#include "URDF/URDFPrefabMaker.h"
#include "URDF/UrdfParser.h"
#include "Utils/RobotImporterUtils.h"
#include <QApplication>
#include <QScreen>
#include <QTranslator>

namespace ROS2
{

    RobotImporterWidget::RobotImporterWidget(QWidget* parent)
        : QWizard(parent)
    {
        m_introPage = new IntroPage(this);
        m_fileSelectPage = new FileSelectionPage(this);
        m_checkUrdfPage = new CheckUrdfPage(this);
        m_assetPage = new CheckAssetPage(this);
        m_prefabMakerPage = new PrefabMakerPage(this);

        addPage(m_introPage);
        addPage(m_fileSelectPage);
        addPage(m_checkUrdfPage);
        addPage(m_assetPage);
        addPage(m_prefabMakerPage);

        connect(this, &QWizard::currentIdChanged, this, &RobotImporterWidget::onCurrentIdChanged);
        connect(m_prefabMakerPage, &QWizardPage::completeChanged, this, &RobotImporterWidget::OnUrdfCreated);
        connect(m_prefabMakerPage, &PrefabMakerPage::onCreateButtonPressed, this, &RobotImporterWidget::onCreateButtonPressed);
        connect(this, &RobotImporterWidget::SignalFinalizeURDFCreation, this, &RobotImporterWidget::FinalizeURDFCreation);
        connect(
            this,
            &QWizard::customButtonClicked,
            this,
            [this](int id)
            {
                if (id == PrefabCreationButtonId)
                {
                    this->onCreateButtonPressed();
                }
            });

        void onCreateButtonPressed();

        setWindowTitle(tr("Robot Import Wizard"));
        connect(
            this,
            &QDialog::finished,
            [this](int id)
            {
                AZ_Printf("page", "QDialog::finished : %d", id);
                parentWidget()->close();
            });
    }

    void RobotImporterWidget::OnUrdfCreated()
    {
        // hide cancel and back buttons when last page succeed
        if (currentPage() == m_prefabMakerPage)
        {
            QWizard::button(QWizard::CancelButton)->hide();
            QWizard::button(QWizard::BackButton)->hide();
            QWizard::button(PrefabCreationButtonId)->hide();
        }
    }

    void RobotImporterWidget::OpenUrdf()
    {
        m_urdfPath = AZStd::string(m_fileSelectPage->getFileName().toUtf8().constData());
        if (!m_urdfPath.empty())
        {
            AZ_Printf("Wizard", "Loading URDF file : %s", m_urdfPath.c_str());
            m_parsedUrdf = UrdfParser::ParseFromFile(m_urdfPath);
            QString report;
            const auto log = UrdfParser::GetUrdfParsingLog();
            if (m_parsedUrdf)
            {
                report += "# " + tr("The URDF was parsed and opened successfully") + "\n";
                // get rid of old prefab maker
                m_prefabMaker.reset();
                // let us skip this page
                AZ_Printf("Wizard", "Wizard skips m_checkUrdfPage since there is no errors in URDF");
                m_meshNames = Utils::GetMeshesFilenames(m_parsedUrdf->getRoot(), true, true);
            }
            else
            {
                report += "# " + tr("The URDF was not opened") + "\n";
                report += tr("URDF parser returned following errors:") + "\n\n";
            }
            if (!log.empty())
            {
                report += "`";
                report += QString::fromUtf8(log.data(), int(log.size()));
                report += "`";
            }
            m_checkUrdfPage->ReportURDFResult(report, m_parsedUrdf != nullptr);
        }
    }

    void RobotImporterWidget::onCurrentIdChanged(int id)
    {
        AZ_Printf("Wizard", "Wizard at page %d", id);

        if (currentPage() == m_assetPage)
        {
            FillAssetPage();
        }
        else if (currentPage() == m_prefabMakerPage)
        {
            FillPrefabMakerPage();
        }
    }

    void RobotImporterWidget::FillAssetPage()
    {
        m_assetPage->ClearAssetsList();
        if (m_parsedUrdf)
        {
            m_urdfAssetsMapping = AZStd::make_shared<Utils::UrdfAssetMap>(Utils::FindAssetsForUrdf(m_meshNames, m_urdfPath));
            auto collidersNames = Utils::GetMeshesFilenames(m_parsedUrdf->getRoot(), false, true);
            auto visualNames = Utils::GetMeshesFilenames(m_parsedUrdf->getRoot(), true, false);
            for (const AZStd::string& meshPath : m_meshNames)
            {
                const QString meshPathqs = QString::fromUtf8(meshPath.data(), meshPath.size());
                const QString kNotFound = tr("not found");

                if (m_urdfAssetsMapping->contains(meshPath))
                {
                    QString type = kNotFound;
                    QString sourcePath = kNotFound;
                    auto crc = AZ::Crc32();
                    QString tooltip = kNotFound;
                    bool visual = visualNames.contains(meshPath);
                    bool collider = collidersNames.contains(meshPath);
                    if (visual && collider)
                    {
                        type = tr("Visual and Collider");
                    }
                    else if (visual)
                    {
                        type = tr("Visual");
                    }
                    else if (collider)
                    {
                        type = tr("Collider");
                    }

                    if (m_urdfAssetsMapping->contains(meshPath))
                    {
                        const auto& asset = m_urdfAssetsMapping->at(meshPath);
                        const AZStd::string& productPath = asset.m_availableAssetInfo.m_productAssetRelativePath;
                        const AZStd::string& resolvedPath = asset.m_resolvedUrdfPath.data();

                        sourcePath = QString::fromUtf8(productPath.data(), productPath.size());
                        crc = asset.m_urdfFileCRC;
                        tooltip = QString::fromUtf8(resolvedPath.data(), resolvedPath.size());
                    }
                    m_assetPage->ReportAsset(meshPathqs, type, sourcePath, crc, tooltip);
                }
                else
                {
                    m_assetPage->ReportAsset(meshPathqs, kNotFound, kNotFound, AZ::Crc32(), kNotFound);
                };
            }
        }
    }

    void RobotImporterWidget::FillPrefabMakerPage()
    {
        if (m_parsedUrdf)
        {
            AZStd::string robotName = AZStd::string(m_parsedUrdf->getName().c_str(), m_parsedUrdf->getName().size()) + ".prefab";
            m_prefabMakerPage->setProposedPrefabName(robotName);
            QWizard::button(PrefabCreationButtonId)->setText(tr("Create Prefab"));
            QWizard::setOption(HavePrefabCreationButton, true);
        }
    }

    bool RobotImporterWidget::validateCurrentPage()
    {
        if (currentPage() == m_fileSelectPage)
        {
            OpenUrdf();
        }
        return currentPage()->validatePage();
    }

    int RobotImporterWidget::nextId() const
    {
        if (currentPage() == m_fileSelectPage)
        {
            if (m_parsedUrdf)
            {
                if (m_meshNames.size() == 0)
                {
                    // skip two pages when urdf is parsed without problems, and it has no meshes
                    return m_assetPage->nextId();
                }
                else
                {
                    // skip one page when urdf is parsed without problems
                    return m_checkUrdfPage->nextId();
                }
            }
        }
        return currentPage()->nextId();
    }

    void RobotImporterWidget::CreatePrefab(AZStd::string prefabName)
    {
        const AZ::IO::Path prefabPathRealative(AZ::IO::Path("Assets") / "Importer" / prefabName);
        const AZ::IO::Path prefabPath(AZ::IO::Path(AZ::Utils::GetProjectPath()) / prefabPathRealative);
        bool fileExists = AZ::IO::FileIOBase::GetInstance()->Exists(prefabPath.c_str());

        if (CheckCyclicalDependency(prefabPathRealative))
        {
            m_prefabMakerPage->setSuccess(false);
            return;
        }
        if (fileExists)
        {
            QMessageBox msgBox;
            msgBox.setText(tr("Prefab with this name already exists"));
            msgBox.setInformativeText(tr("Do you want to overwrite existing prefab?"));
            msgBox.setStandardButtons(QMessageBox::Ok | QMessageBox::Cancel);
            msgBox.setDefaultButton(QMessageBox::Cancel);
            int ret = msgBox.exec();
            if (ret == QMessageBox::Cancel)
            {
                m_prefabMakerPage->setSuccess(false);
                return;
            }
        }
        m_prefabMaker = AZStd::make_unique<URDFPrefabMaker>(m_urdfPath, m_parsedUrdf, prefabPath.String(), m_urdfAssetsMapping);

        auto callback = [&]()
        {
            emit SignalFinalizeURDFCreation();
        };
        m_prefabMaker->LoadURDF(callback);
    }
    void RobotImporterWidget::FinalizeURDFCreation()
    {
        auto prefabOutcome = m_prefabMaker->CreatePrefabFromURDF();
        if (prefabOutcome.IsSuccess())
        {
            AZStd::string status = m_prefabMaker->getStatus();
            m_prefabMakerPage->reportProgress(status);
            m_prefabMakerPage->setSuccess(true);
        }
        else
        {
            AZStd::string status = "Failed to create prefab\n";
            status += prefabOutcome.GetError() + "\n";
            status += m_prefabMaker->getStatus();
            m_prefabMakerPage->reportProgress(status);
            m_prefabMakerPage->setSuccess(false);
        }
    }
    void RobotImporterWidget::onCreateButtonPressed()
    {
        CreatePrefab(m_prefabMakerPage->getPrefabName());
    }

    bool RobotImporterWidget::CheckCyclicalDependency(AZ::IO::Path importedPrefabPath)
    {
        AzFramework::EntityContextId contextId;
        AzFramework::EntityIdContextQueryBus::BroadcastResult(contextId, &AzFramework::EntityIdContextQueryBus::Events::GetOwningContextId);

        AZ_Printf("CheckCyclicalDependency", "CheckCyclicalDependency %s\n", importedPrefabPath.Native().c_str());
        auto focusInterface = AZ::Interface<AzToolsFramework::Prefab::PrefabFocusInterface>::Get();

        if (!focusInterface)
        {
            ReportError(tr("Imported prefab could not be validated.\nImport aborted."));
            return true;
        }

        auto focusedPrefabInstance = focusInterface->GetFocusedPrefabInstance(contextId);

        if (!focusedPrefabInstance)
        {
            ReportError(tr("Imported prefab could not be validated.\nImport aborted."));
            return true;
        }

        auto focusPrefabFilename = focusedPrefabInstance.value().get().GetTemplateSourcePath();

        if (focusPrefabFilename == importedPrefabPath)
        {
            ReportError(
                tr("Cyclical dependency detected.\nSelected URDF model is currently being edited. Exit prefab edit mode and try again."));
            return true;
        }

        return false;
    }

    void RobotImporterWidget::ReportError(const QString& errorMessage)
    {
        QMessageBox::critical(this, QObject::tr("Error"), errorMessage);
        AZ_Error("RobotImporterWidget", false, "%s", errorMessage.toUtf8().constData());
    }
} // namespace ROS2
