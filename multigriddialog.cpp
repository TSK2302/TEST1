#include "multigriddialog.h"
#include "ui_multigriddialog.h"
#include <QDoubleSpinBox>
#include <QVBoxLayout>

MultiGridDialog::MultiGridDialog(int rows, int columns, float cellSize, const QString& name, QWidget *parent) :
    QDialog(parent),
    ui(new Ui::MultiGridDialog),
    m_rows(rows),
    m_columns(columns),
    m_cellSize(cellSize),
    m_baseName(name)
{
    ui->setupUi(this);

    // Set window properties
    setWindowTitle(tr("Duplicate 2D Grid"));

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
        "QScrollArea {"
        "    background-color: #3C3C3C;"
        "    border: 1px solid #555555;"
        "}"
    );

    // Connect duplicate count change to update height fields
    connect(ui->duplicateCountSpinBox, QOverload<int>::of(&QSpinBox::valueChanged),
            this, &MultiGridDialog::on_duplicateCountSpinBox_valueChanged);

    // Initialize with one height field starting at 0
    ui->duplicateCountSpinBox->setValue(1);
    on_duplicateCountSpinBox_valueChanged(1);
}

MultiGridDialog::~MultiGridDialog()
{
    delete ui;
}

int MultiGridDialog::getDuplicateCount() const
{
    return ui->duplicateCountSpinBox->value();
}

std::vector<float> MultiGridDialog::getHeights() const
{
    std::vector<float> heights;
    QVBoxLayout* heightsLayout = qobject_cast<QVBoxLayout*>(ui->scrollAreaWidgetContents->layout());
    for (int i = 0; i < heightsLayout->count(); ++i) {
        QDoubleSpinBox* spinBox = qobject_cast<QDoubleSpinBox*>(heightsLayout->itemAt(i)->widget());
        if (spinBox) {
            heights.push_back(static_cast<float>(spinBox->value()));
        }
    }
    return heights;
}

void MultiGridDialog::on_buttonBox_accepted()
{
    emit multiGridParametersConfirmed(m_rows, m_columns, m_cellSize, getDuplicateCount(), getHeights(), m_baseName);
    accept();
}

void MultiGridDialog::on_buttonBox_rejected()
{
    reject();
}

void MultiGridDialog::on_duplicateCountSpinBox_valueChanged(int value)
{
    QVBoxLayout* heightsLayout = qobject_cast<QVBoxLayout*>(ui->scrollAreaWidgetContents->layout());
    if (!heightsLayout) return;

    // Clear existing spin boxes
    while (QLayoutItem* item = heightsLayout->takeAt(0)) {
        delete item->widget();
        delete item;
    }

    // Add new spin boxes based on duplicate count
    for (int i = 0; i < value; ++i) {
        QDoubleSpinBox* heightSpinBox = new QDoubleSpinBox(ui->scrollAreaWidgetContents);
        heightSpinBox->setObjectName(QString("heightSpinBox_%1").arg(i + 1));
        heightSpinBox->setMinimum(-1000.0);
        heightSpinBox->setMaximum(1000.0);
        heightSpinBox->setValue(i * 1.0); // Default heights: 0, 1, 2, ...
        heightsLayout->addWidget(heightSpinBox);
    }

    // Add stretch to keep spin boxes at top
    heightsLayout->addStretch();
}