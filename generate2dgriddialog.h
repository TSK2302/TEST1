#ifndef GENERATE2DGRID_H
#define GENERATE2DGRID_H

#include <QDialog>

namespace Ui {
class Generate2DGridDialog;
}

class Generate2DGridDialog : public QDialog
{
    Q_OBJECT

public:
    explicit Generate2DGridDialog(QWidget *parent = nullptr);
    ~Generate2DGridDialog();

    int getRows() const;
    int getColumns() const;
    float getCellSize() const;

signals:
    void gridParametersConfirmed(int rows, int columns, float cellSize);

private slots:
    void on_buttonBox_accepted();
    void on_buttonBox_rejected();

    // void on_columnsSpinBox_textChanged(const QString &arg1);

private:
    Ui::Generate2DGridDialog *ui;
};

#endif // GENERATE2DGRID_H
