#ifndef COLORCHANGER_H
#define COLORCHANGER_H

#include <QDialog>
#include <QColorDialog>
#include <QVector3D>

namespace Ui {
class ColorChanger;
}

class ColorChanger : public QDialog
{
    Q_OBJECT

public:
    explicit ColorChanger(QWidget *parent = nullptr);
    ~ColorChanger();

signals:
    void applySingleColor(const QString& pointCloudName, const QVector3D& color);
    void applyGradientColor(const QString& pointCloudName, const QVector3D& startColor, const QVector3D& endColor);

public slots:
    void setPointCloudName(const QString& name);

private slots:
    void on_singleColorButton_clicked();
    void on_gradientStartButton_clicked();
    void on_gradientEndButton_clicked();
    void on_applyButton_clicked();

private:
    Ui::ColorChanger *ui;
    QString m_pointCloudName;
    QColor m_singleColor;
    QColor m_gradientStartColor;
    QColor m_gradientEndColor;
};

#endif // COLORCHANGER_H