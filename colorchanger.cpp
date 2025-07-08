#include "colorchanger.h"
#include "ui_colorchanger.h"
#include <QColorDialog>

ColorChanger::ColorChanger(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::ColorChanger),
    m_singleColor(Qt::white),
    m_gradientStartColor(Qt::blue),
    m_gradientEndColor(Qt::red)
{
    ui->setupUi(this);
}

ColorChanger::~ColorChanger()
{
    delete ui;
}

void ColorChanger::setPointCloudName(const QString& name)
{
    m_pointCloudName = name;
}

void ColorChanger::on_singleColorButton_clicked()
{
    QColor color = QColorDialog::getColor(m_singleColor, this, "Select Color");
    if (color.isValid()) {
        m_singleColor = color;
    }
}

void ColorChanger::on_gradientStartButton_clicked()
{
    QColor color = QColorDialog::getColor(m_gradientStartColor, this, "Select Gradient Start Color");
    if (color.isValid()) {
        m_gradientStartColor = color;
    }
}

void ColorChanger::on_gradientEndButton_clicked()
{
    QColor color = QColorDialog::getColor(m_gradientEndColor, this, "Select Gradient End Color");
    if (color.isValid()) {
        m_gradientEndColor = color;
    }
}

void ColorChanger::on_applyButton_clicked()
{
    if (ui->singleColorRadio->isChecked()) {
        QVector3D color(m_singleColor.redF(), m_singleColor.greenF(), m_singleColor.blueF());
        emit applySingleColor(m_pointCloudName, color);
    } else {
        QVector3D startColor(m_gradientStartColor.redF(), m_gradientStartColor.greenF(), m_gradientStartColor.blueF());
        QVector3D endColor(m_gradientEndColor.redF(), m_gradientEndColor.greenF(), m_gradientEndColor.blueF());
        emit applyGradientColor(m_pointCloudName, startColor, endColor);
    }
    accept();
}