#ifndef MANAGETWINS_H
#define MANAGETWINS_H

#include <QDialog>
#include <QDir>

namespace Ui {
class ManageTwins;
}

class ManageTwins : public QDialog
{
    Q_OBJECT

public:
    explicit ManageTwins(QWidget *parent = nullptr);
    ~ManageTwins();

private slots:
    void on_selectFolderButton_clicked();
    void on_addFileButton_clicked();
    void on_deleteFileButton_clicked();
    void on_renameFileButton_clicked();

private:
    Ui::ManageTwins *ui;
    QDir currentDir;

    void updateFileList();
};

#endif // MANAGETWINS_H