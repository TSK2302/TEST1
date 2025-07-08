#ifndef MANAGEOBJECTS_H
#define MANAGEOBJECTS_H

#include <QDialog>
#include <QDir>

namespace Ui {
class ManageObjects;
}

class ManageObjects : public QDialog
{
    Q_OBJECT

public:
    explicit ManageObjects(QWidget *parent = nullptr);
    ~ManageObjects();

private slots:
    void on_selectFolderButton_clicked();
    void on_addFileButton_clicked();
    void on_deleteFileButton_clicked();
    void on_renameFileButton_clicked();

private:
    Ui::ManageObjects *ui;
    QDir currentDir;

    void updateFileList();
};

#endif // MANAGEOBJECTS_H