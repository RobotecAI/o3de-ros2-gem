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

#include "RobotImporter/RobotImporterWidget.h"
#include "RobotImporter/RobotImporterWidgetUtils.h"
#include "RobotImporter/URDF/URDFPrefabMaker.h"
#include "RobotImporter/URDF/UrdfParser.h"
#include "RobotImporter/Utils/RobotImporterUtils.h"
#include <QApplication>
#include <QScreen>

namespace ROS2
{
    QWizardPage* createIntroPage()
    {
        QWizardPage* page = new QWizardPage;
        page->setTitle("Introduction");

        QLabel* label = new QLabel("This wizard allows you to build robot simulation"
                                   "out of URDF description."
                                   "<br> before proceed please make that all assets are imported</br>");
        label->setWordWrap(true);

        QVBoxLayout* layout = new QVBoxLayout;
        layout->addWidget(label);
        page->setLayout(layout);
        return page;
    }

    FileSelectionPage::FileSelectionPage(QWizard* parent)
        : QWizardPage(parent)
    {
        m_fileDialog = new QFileDialog(this);
        m_fileDialog->setDirectory(QString::fromUtf8(AZ::Utils::GetProjectPath().data()));
        m_fileDialog->setNameFilter("URDF (*.urdf *.udf)");
        m_button = new QPushButton("...", this);
        // TODO remove this!
        m_textEdit = new QLineEdit("", this);
        setTitle("Load URDF file");
        QVBoxLayout* layout = new QVBoxLayout;
        layout->addStretch();
        layout->addWidget(new QLabel("URDF file : ", this));
        QHBoxLayout* layout_in = new QHBoxLayout;
        layout_in->addWidget(m_button);
        layout_in->addWidget(m_textEdit);
        layout->addLayout(layout_in);
        layout->addStretch();
        this->setLayout(layout);
        connect(m_button, &QPushButton::pressed, this, &FileSelectionPage::onLoadButtonPressed);
        connect(m_fileDialog, &QFileDialog::fileSelected, this, &FileSelectionPage::onFileSelected);
        connect(m_textEdit, &QLineEdit::textChanged, this, &FileSelectionPage::onTextChanged);
        FileSelectionPage::onTextChanged(m_textEdit->text());
    }

    void FileSelectionPage::onLoadButtonPressed()
    {
        m_fileDialog->show();
    }
    void FileSelectionPage::onFileSelected(const QString& file)
    {
        m_textEdit->setText(file);
    }
    void FileSelectionPage::onTextChanged(const QString& text)
    {
        m_fileExists = QFileInfo::exists(text);
        emit completeChanged();
    }
    bool FileSelectionPage::isComplete() const
    {
        return m_fileExists;
    };

    CheckUrdfPage::CheckUrdfPage(QWizard* parent)
        : QWizardPage(parent)
        , m_success(false)
    {
        m_log = new QTextEdit(this);
        setTitle("URDF opening results:");
        QVBoxLayout* layout = new QVBoxLayout;
        layout->addWidget(m_log);
        m_log->acceptRichText();
        m_log->setReadOnly(true);
        this->setLayout(layout);
    }

    void CheckUrdfPage::ReportURDFResult(const QString& status, bool is_success)
    {
        m_log->setMarkdown(status);
        m_success = is_success;
        emit completeChanged();
    }

    bool CheckUrdfPage::isComplete() const
    {
        return m_success;
    }

    CheckAssetPage::CheckAssetPage(QWizard* parent)
        : QWizardPage(parent)
        , m_success(true)
        , m_missingCount(0)
    {
        m_table = new QTableWidget(parent);
        SetTitle();
        QVBoxLayout* layout = new QVBoxLayout;
        layout->addWidget(m_table);
        m_table->setColumnCount(4);
        m_table->horizontalHeader()->setSectionResizeMode(0, QHeaderView::Stretch);
        m_table->setShowGrid(true);
        m_table->setSelectionMode(QAbstractItemView::SingleSelection);
        m_table->setSelectionBehavior(QAbstractItemView::SelectRows);
        m_table->setHorizontalHeaderLabels({ "URDF mesh path", "CRC", "Type", "Asset source" });
        m_table->horizontalHeader()->setStretchLastSection(true);
        this->setLayout(layout);
    }

    void CheckAssetPage::SetTitle()
    {
        if (m_missingCount == 0)
        {
            setTitle("Resolved meshes");
        }
        else
        {
            setTitle("There are " + QString::number(m_missingCount) + " unresolved meshes");
        }
    }

    bool CheckAssetPage::isComplete() const
    {
        return m_success;
    };

    void CheckAssetPage::ReportAsset(
        const AZStd::string& urdfPath,
        const AZStd::string& type,
        const AZStd::string& assetSourcePath,
        const AZ::Crc32& crc32,
        const AZStd::string& tooltip)
    {
        int i = m_table->rowCount();
        m_table->setRowCount(i + 1);

        bool isOk = !assetSourcePath.empty();
        if (!isOk)
        {
            m_missingCount++;
        }
        SetTitle();
        QString qtooltip = QString::fromUtf8(tooltip.data(), int(tooltip.size()));

        QTableWidgetItem* p = createCell(isOk, urdfPath);
        p->setToolTip("Resolved to : " + qtooltip);
        m_table->setItem(i, 0, p);
        m_table->setItem(i, 1, createCell(isOk, AZStd::to_string(crc32)));
        m_table->setItem(i, 2, createCell(isOk, type));
        m_table->setItem(i, 3, createCell(isOk, assetSourcePath));
    }

    QTableWidgetItem* CheckAssetPage::createCell(bool isOk, const AZStd::string& text)
    {
        QTableWidgetItem* p = new QTableWidgetItem(QString::fromUtf8(text.data(), int(text.size())));
        if (!isOk)
        {
            p->setBackground(Qt::red);
        }
        p->setFlags(Qt::NoItemFlags);
        return p;
    }

    void CheckAssetPage::ClearAssetsList()
    {
        m_table->setRowCount(0);
        m_missingCount = 0;
    }

    PrefabMakerPage::PrefabMakerPage(RobotImporterWidget* parent)
        : QWizardPage(parent)
        , m_parentImporterWidget(parent)
        , m_success(false)
    {
        m_prefabName = new QLineEdit(this);
        m_createButton = new QPushButton("Create Prefab", this);
        m_log = new QTextEdit(this);
        setTitle("Prefab creation");
        QVBoxLayout* layout = new QVBoxLayout;
        QHBoxLayout* layoutInner = new QHBoxLayout;
        layoutInner->addWidget(m_prefabName);
        layoutInner->addWidget(m_createButton);
        layout->addLayout(layoutInner);
        layout->addWidget(m_log);
        this->setLayout(layout);
        connect(m_createButton, &QPushButton::pressed, this, &PrefabMakerPage::onCreateButtonPressed);
    }

    void PrefabMakerPage::setProposedPrefabName(const AZStd::string prefabName)
    {
        m_prefabName->setText(QString::fromUtf8(prefabName.data(), int(prefabName.size())));
    }

    AZStd::string PrefabMakerPage::getPrefabName() const
    {
        return AZStd::string(m_prefabName->text().toUtf8().constData());
    }

    void PrefabMakerPage::reportProgress(const AZStd::string& progressForUser)
    {
        m_log->setText(QString::fromUtf8(progressForUser.data(), int(progressForUser.size())));
    }
    void PrefabMakerPage::setSuccess(bool success)
    {
        m_success = success;
        emit completeChanged();
    }
    bool PrefabMakerPage::isComplete() const
    {
        return m_success;
    }

    RobotImporterWidget::RobotImporterWidget(QWidget* parent)
        : QWizard(parent)
    {
        m_fileSelectPage = new FileSelectionPage(this);
        m_checkUrdfPage = new CheckUrdfPage(this);
        m_assetPage = new CheckAssetPage(this);
        m_prefabMakerPage = new PrefabMakerPage(this);

        addPage(createIntroPage());
        addPage(m_fileSelectPage);
        addPage(m_checkUrdfPage);
        addPage(m_assetPage);
        addPage(m_prefabMakerPage);

        connect(this, &QWizard::currentIdChanged, this, &RobotImporterWidget::onCurrentIdChanged);
        connect(m_prefabMakerPage, &QWizardPage::completeChanged, this, &RobotImporterWidget::OnUrdfCreated);
        connect(m_prefabMakerPage, &PrefabMakerPage::onCreateButtonPressed, this, &RobotImporterWidget::onCreateButtonPressed);
        connect(
            this,
            &QWizard::customButtonClicked,
            this,
            [this](int id)
            {
                if (id == QWizard::CustomButton1)
                {
                    this->onCreateButtonPressed();
                }
            });

        void onCreateButtonPressed();

        setWindowTitle("Robot Import Wizard");
        connect(
            this,
            &QDialog::finished,
            [this](int id)
            {
                AZ_Printf("page", "QDialog::finished : %d", id);
                parentWidget()->close();
                //                      AzToolsFramework::CloseViewPane("Robot Importer");
            });
    }

    void RobotImporterWidget::OnUrdfCreated()
    {
        // hide cancel and back buttons when last page succeed
        if (currentPage() == m_prefabMakerPage)
        {
            QWizard::button(QWizard::CancelButton)->hide();
            QWizard::button(QWizard::BackButton)->hide();
            QWizard::button(QWizard::CustomButton1)->hide();
        }
    }

    void RobotImporterWidget::OpenUrdf()
    {
        m_urdfPath = AZStd::string(m_fileSelectPage->getFileName().toUtf8().constData());
        if (!m_urdfPath.empty())
        {
            AZ_Printf("Wizard", "Testing urdf file : %s", m_urdfPath.c_str());
            m_parsedUrdf = UrdfParser::ParseFromFile(m_urdfPath);
            QString report;
            const auto log = UrdfParser::getUrdfParsingLog();
            if (m_parsedUrdf)
            {
                report += "# The URDF was parsed and opened successfully\n";
                // get rid of old prefab maker
                m_prefabMaker.reset();
                // let us skip this page
                AZ_Printf("Wizard", "Wizard skips m_checkUrdfPage since there is no errors in URDF");
                m_meshNames = Utils::getMeshesFilenames(m_parsedUrdf->getRoot(), true, true);
            }
            else
            {
                report += "# The URDF was not opened\n";
                report += "URDF parser returned following errors:\n\n";
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
            m_assetPage->ClearAssetsList();
            if (m_parsedUrdf)
            {
                m_urdfAssetsMapping = AZStd::make_shared<AZStd::unordered_map<AZStd::string, Utils::urdf_asset>>(
                    Utils::findAssetsForUrdf(m_meshNames, m_urdfPath));
                auto colliders_names = Utils::getMeshesFilenames(m_parsedUrdf->getRoot(), false, true);
                auto visual_names = Utils::getMeshesFilenames(m_parsedUrdf->getRoot(), true, false);
                for (auto& mesh_path : m_meshNames)
                {
                    const AZStd::string kNotFound = "not found";

                    if (m_urdfAssetsMapping->contains(mesh_path))
                    {
                        AZStd::string type = kNotFound;
                        AZStd::string source_path = kNotFound;
                        auto crc = AZ::Crc32();
                        AZStd::string tooltip = kNotFound;
                        if (visual_names.contains(mesh_path))
                        {
                            type = "Visual";
                        }
                        else if (colliders_names.contains(mesh_path))
                        {
                            type = "Collider";
                        }
                        if (m_urdfAssetsMapping->contains(mesh_path))
                        {
                            const auto& asset = m_urdfAssetsMapping->at(mesh_path);
                            source_path = asset.m_availableAssetInfo.m_productAssetRelativePath;
                            crc = asset.m_urdfFileCRC;
                            tooltip = asset.m_resolvedUrdfPath;
                        }
                        m_assetPage->ReportAsset(mesh_path, type, source_path, crc, tooltip);
                    }
                    else
                    {
                        m_assetPage->ReportAsset(mesh_path, kNotFound, kNotFound, AZ::Crc32(), kNotFound);
                    };
                }
            }
        }
        else if (currentPage() == m_prefabMakerPage)
        {
            if (m_parsedUrdf)
            {
                AZStd::string robotName = AZStd::string(m_parsedUrdf->getName().c_str(), m_parsedUrdf->getName().size()) + ".prefab";
                m_prefabMakerPage->setProposedPrefabName(robotName);
                QWizard::button(QWizard::CustomButton1)->setText("Create Prefab");
                QWizard::setOption(QWizard::HaveCustomButton1, true);
            }
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
        const AZ::IO::Path prefabPath(AZ::IO::Path(AZ::Utils::GetProjectPath()) / "Assets" / "Importer" / prefabName);
        bool fileExists = AZ::IO::FileIOBase::GetInstance()->Exists(prefabPath.c_str());
        if (fileExists)
        {
            QMessageBox msgBox;
            msgBox.setText("Prefab with this name already exists");
            msgBox.setInformativeText("Do you want to overwrite existing prefab ?");
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


    bool RobotImporterWidget::CheckCyclicalDependency(const AZ::IO::PathView& importedPrefabFilename)
    {
        AzFramework::EntityContextId context_id;
        EBUS_EVENT_RESULT(context_id, AzFramework::EntityIdContextQueryBus, GetOwningContextId);

        auto focus_interface = AZ::Interface<AzToolsFramework::Prefab::PrefabFocusInterface>::Get();

        if (!focus_interface)
        {
            ReportError("Imported prefab could not be validated.\nImport aborted.");
            return true;
        }

        auto focus_prefab_instance = focus_interface->GetFocusedPrefabInstance(context_id);

        if (!focus_prefab_instance)
        {
            ReportError("Imported prefab could not be validated.\nImport aborted.");
            return true;
        }

        auto focus_prefab_filename = focus_prefab_instance.value().get().GetTemplateSourcePath().Filename();

        if (focus_prefab_filename == importedPrefabFilename)
        {
            ReportError(
                    "Cyclical dependency detected.\nSelected URDF model is currently being edited. Exit prefab edit mode and try again.");
            return true;
        }

        return false;
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
