#include "convert2dto3dgriddialog.h"
#include "ui_convert2dto3dgriddialog.h"
#include <QMessageBox>

Convert2DTo3DGridDialog::Convert2DTo3DGridDialog(int rows, int columns, float cellSize, const QString& gridName, QWidget *parent)
    : QDialog(parent)
    , ui(new Ui::Convert2DTo3DGridDialog)
    , m_rows(rows)
    , m_columns(columns)
    , m_cellSize(cellSize)
    , m_gridName(gridName)
{
    ui->setupUi(this);

    // Pre-fill fields with 2D grid parameters
    ui->cellSizeEdit->setText(QString::number(m_cellSize));
    ui->layersEdit->setText("10"); // Default layers
}

Convert2DTo3DGridDialog::~Convert2DTo3DGridDialog()
{
    delete ui;
}

void Convert2DTo3DGridDialog::on_okButton_clicked()
{
    bool ok;
    int layers = ui->layersEdit->text().toInt(&ok);
    if (!ok || layers <= 0) {
        QMessageBox::warning(this, tr("Invalid Input"), tr("Please enter a valid number of layers (positive integer)."));
        return;
    }

    float cellSize = ui->cellSizeEdit->text().toFloat(&ok);
    if (!ok || cellSize <= 0.0f) {
        QMessageBox::warning(this, tr("Invalid Input"), tr("Please enter a valid cell size (positive number)."));
        return;
    }

    emit gridParametersConfirmed(m_rows, m_columns, layers, cellSize, m_gridName);
    accept();
}