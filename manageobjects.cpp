#include "manageobjects.h"
#include "ui_manageobjects.h"
#include <QFileDialog>
#include <QMessageBox>
#include <QInputDialog>
#include <QDebug>

ManageObjects::ManageObjects(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::ManageObjects)
{
    ui->setupUi(this);
    updateFileList();
}

ManageObjects::~ManageObjects()
{
    delete ui;
}

void ManageObjects::updateFileList()
{
    ui->fileListWidget->clear();
    if (currentDir.exists()) {
        QStringList filters;
        filters << "*.pts" << "*.ply" << "*.obj" << "*.fbx" << "*.stl" << "*.dae" 
                << "*.gltf" << "*.glb";
        QFileInfoList fileList = currentDir.entryInfoList(filters, QDir::Files);
        for (const QFileInfo &fileInfo : fileList) {
            ui->fileListWidget->addItem(fileInfo.fileName());
        }
        ui->folderPathEdit->setText(currentDir.absolutePath());
    } else {
        ui->folderPathEdit->clear();
    }
}

void ManageObjects::on_selectFolderButton_clicked()
{
    QString dirPath = QFileDialog::getExistingDirectory(
        this,
        tr("Select Folder"),
        QDir::homePath(),
        QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks
    );
    
    if (!dirPath.isEmpty()) {
        currentDir.setPath(dirPath);
        updateFileList();
    }
}

void ManageObjects::on_addFileButton_clicked()
{
    if (!currentDir.exists()) {
        QMessageBox::warning(this, tr("No Folder Selected"), 
                            tr("Please select a folder first."));
        return;
    }

    QStringList filePaths = QFileDialog::getOpenFileNames(
        this,
        tr("Add Files"),
        QDir::homePath(),
        tr("3D Files (*.pts *.ply *.obj *.fbx *.stl *.dae *.gltf *.glb)")
    );

    for (const QString &filePath : filePaths) {
        QFileInfo fileInfo(filePath);
        QString destPath = currentDir.absoluteFilePath(fileInfo.fileName());
        
        if (QFile::exists(destPath)) {
            QMessageBox::warning(this, tr("File Exists"),
                               tr("File %1 already exists in the folder.").arg(fileInfo.fileName()));
            continue;
        }

        if (!QFile::copy(filePath, destPath)) {
            QMessageBox::critical(this, tr("Error"),
                                 tr("Failed to copy file %1.").arg(fileInfo.fileName()));
        }
    }
    
    updateFileList();
}

void ManageObjects::on_deleteFileButton_clicked()
{
    if (!currentDir.exists()) {
        QMessageBox::warning(this, tr("No Folder Selected"), 
                            tr("Please select a folder first."));
        return;
    }

    QList<QListWidgetItem*> selectedItems = ui->fileListWidget->selectedItems();
    if (selectedItems.isEmpty()) {
        QMessageBox::warning(this, tr("No Selection"), 
                            tr("Please select at least one file to delete."));
        return;
    }

    QMessageBox::StandardButton reply = QMessageBox::question(
        this,
        tr("Confirm Delete"),
        tr("Are you sure you want to delete the selected file(s)?"),
        QMessageBox::Yes | QMessageBox::No
    );

    if (reply == QMessageBox::Yes) {
        for (QListWidgetItem* item : selectedItems) {
            QString filePath = currentDir.absoluteFilePath(item->text());
            if (!QFile::remove(filePath)) {
                QMessageBox::critical(this, tr("Error"),
                                    tr("Failed to delete file %1.").arg(item->text()));
            }
        }
        updateFileList();
    }
}

void ManageObjects::on_renameFileButton_clicked()
{
    if (!currentDir.exists()) {
        QMessageBox::warning(this, tr("No Folder Selected"), 
                            tr("Please select a folder first."));
        return;
    }

    QList<QListWidgetItem*> selectedItems = ui->fileListWidget->selectedItems();
    if (selectedItems.size() != 1) {
        QMessageBox::warning(this, tr("Invalid Selection"), 
                            tr("Please select exactly one file to rename."));
        return;
    }

    QString oldName = selectedItems.first()->text();
    bool ok;
    QString newName = QInputDialog::getText(
        this,
        tr("Rename File"),
        tr("New file name:"),
        QLineEdit::Normal,
        oldName,
        &ok
    );

    if (ok && !newName.isEmpty()) {
        // Validate file extension
        QStringList validExtensions = {".pts", ".ply", ".obj", ".fbx", ".stl", ".dae", ".gltf", ".glb"};
        bool validExtension = false;
        for (const QString &ext : validExtensions) {
            if (newName.endsWith(ext, Qt::CaseInsensitive)) {
                validExtension = true;
                break;
            }
        }

        if (!validExtension) {
            QMessageBox::warning(this, tr("Invalid Extension"),
                                tr("File must have one of the following extensions: %1")
                                .arg(validExtensions.join(", ")));
            return;
        }

        QString oldPath = currentDir.absoluteFilePath(oldName);
        QString newPath = currentDir.absoluteFilePath(newName);

        if (QFile::exists(newPath)) {
            QMessageBox::warning(this, tr("File Exists"),
                                tr("A file named %1 already exists.").arg(newName));
            return;
        }

        if (!QFile::rename(oldPath, newPath)) {
            QMessageBox::critical(this, tr("Error"),
                                 tr("Failed to rename file %1 to %2.").arg(oldName, newName));
        } else {
            updateFileList();
        }
    }
}