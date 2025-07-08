#ifndef CONVERT2DTO3DGRIDDIALOG_H
#define CONVERT2DTO3DGRIDDIALOG_H

#include <QDialog>

namespace Ui {
class Convert2DTo3DGridDialog;
}

class Convert2DTo3DGridDialog : public QDialog
{
    Q_OBJECT

public:
    explicit Convert2DTo3DGridDialog(int rows, int columns, float cellSize, const QString& gridName, QWidget *parent = nullptr);
    ~Convert2DTo3DGridDialog();

signals:
    void gridParametersConfirmed(int rows, int columns, int layers, float cellSize, const QString& gridName);

private slots:
    void on_okButton_clicked();

private:
    Ui::Convert2DTo3DGridDialog *ui;
    int m_rows;
    int m_columns;
    float m_cellSize;
    QString m_gridName;
};

#endif // CONVERT2DTO3DGRIDDIALOG_H