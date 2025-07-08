#include "generate2dgriddialog.h"
#include "ui_generate2dgriddialog.h"

Generate2DGridDialog::Generate2DGridDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::Generate2DGridDialog)
{
    ui->setupUi(this);

    // Set window properties
    setWindowTitle(tr("Generate 2D Grid"));

    // Apply dark theme via stylesheet
    setStyleSheet(
        "QDialog {"
        "    background-color: #2E2E2E;"
        "    color: #FFFFFF;"
        "}"
        "QLabel {"
        "    color: #FFFFFF;"
        "    font-size: 12px;"
        "}"
        "QSpinBox, QDoubleSpinBox {"
        "    background-color: #3C3C3C;"
        "    color: #FFFFFF;"
        "    border: 1px solid #555555;"
        "    padding: 4px;"
        "    border-radius: 3px;"
        "}"
        "QSpinBox::up-button, QSpinBox::down-button,"
        "QDoubleSpinBox::up-button, QDoubleSpinBox::down-button {"
        "    background-color: #555555;"
        "    border: none;"
        "}"
        "QPushButton {"
        "    background-color: #4A4A4A;"
        "    color: #FFFFFF;"
        "    border: 1px solid #555555;"
        "    padding: 5px 10px;"
        "    border-radius: 3px;"
        "}"
        "QPushButton:hover {"
        "    background-color: #5A5A5A;"
        "}"
        "QPushButton:pressed {"
        "    background-color: #3A3A3A;"
        "}"
        );
}

Generate2DGridDialog::~Generate2DGridDialog()
{
    delete ui;
}

int Generate2DGridDialog::getRows() const
{
    return ui->rowsSpinBox->value();
}

int Generate2DGridDialog::getColumns() const
{
    return ui->columnsSpinBox->value();
}

float Generate2DGridDialog::getCellSize() const
{
    return static_cast<float>(ui->cellSizeSpinBox->value());
}

void Generate2DGridDialog::on_buttonBox_accepted()
{
    emit gridParametersConfirmed(getRows(), getColumns(), getCellSize());
    accept();
}

void Generate2DGridDialog::on_buttonBox_rejected()
{
    reject();
}
