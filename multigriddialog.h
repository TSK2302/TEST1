#ifndef MULTIGRIDDIALOG_H
#define MULTIGRIDDIALOG_H

#include <QDialog>

namespace Ui {
class MultiGridDialog;
}

class MultiGridDialog : public QDialog
{
    Q_OBJECT

public:
    explicit MultiGridDialog(int rows, int columns, float cellSize, const QString& name, QWidget *parent = nullptr);
    ~MultiGridDialog();

    int getDuplicateCount() const;
    std::vector<float> getHeights() const;

signals:
    void multiGridParametersConfirmed(int rows, int columns, float cellSize, int duplicateCount, const std::vector<float>& heights, const QString& baseName);

private slots:
    void on_buttonBox_accepted();
    void on_buttonBox_rejected();
    void on_duplicateCountSpinBox_valueChanged(int value);

private:
    Ui::MultiGridDialog *ui;
    int m_rows;
    int m_columns;
    float m_cellSize;
    QString m_baseName;
};

#endif // MULTIGRIDDIALOG_H