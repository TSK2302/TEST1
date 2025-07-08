#include "generate3dgriddialog.h"
#include "ui_generate3dgriddialog.h"
#include <QMessageBox>

Generate3DGridDialog::Generate3DGridDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::Generate3DGridDialog)
{
    ui->setupUi(this);
}

Generate3DGridDialog::~Generate3DGridDialog()
{
    delete ui;
}

void Generate3DGridDialog::on_pushButtonOk_clicked()
{
    bool okRows, okCols, okLayers, okSize;
    int rows = ui->lineEditRows->text().toInt(&okRows);
    int columns = ui->lineEditColumns->text().toInt(&okCols);
    int layers = ui->lineEditLayers->text().toInt(&okLayers);
    float cellSize = ui->lineEditCellSize->text().toFloat(&okSize);

    if (!okRows || !okCols || !okLayers || !okSize || rows <= 0 || columns <= 0 || layers <= 0 || cellSize <= 0.0f) {
        QMessageBox::warning(this, tr("Invalid Input"), tr("Please enter valid positive numbers for rows, columns, layers, and cell size."));
        return;
    }

    emit gridParametersConfirmed(rows, columns, layers, cellSize);
    accept();
}