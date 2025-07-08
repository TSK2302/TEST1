#ifndef GENERATE3DGRIDDIALOG_H
#define GENERATE3DGRIDDIALOG_H

#include <QDialog>

namespace Ui {
class Generate3DGridDialog;
}

class Generate3DGridDialog : public QDialog
{
    Q_OBJECT

public:
    explicit Generate3DGridDialog(QWidget *parent = nullptr);
    ~Generate3DGridDialog();

signals:
    void gridParametersConfirmed(int rows, int columns, int layers, float cellSize);

private slots:
    void on_pushButtonOk_clicked();

private:
    Ui::Generate3DGridDialog *ui;
};

#endif // GENERATE3DGRIDDIALOG_H