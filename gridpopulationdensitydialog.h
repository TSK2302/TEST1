#ifndef GRIDPOPULATIONDENSITYDIALOG_H
#define GRIDPOPULATIONDENSITYDIALOG_H

#include <QDialog>
#include <QGridLayout>
#include <QPushButton>
#include <QVector>
#include <vector>
#include "openglwidget.h"

namespace Ui {
class GridPopulationDensityDialog;
}

class GridPopulationDensityDialog : public QDialog
{
    Q_OBJECT

public:
    explicit GridPopulationDensityDialog(const OpenGLWidget::Grid2D& grid, 
                                        const QString& pointCloudName, 
                                        OpenGLWidget* openGLWidget,
                                        QWidget *parent = nullptr);
    ~GridPopulationDensityDialog();

    void setStatusMessage(const QString& message);
    void updateLegend();

public slots:
    void onPointCloudTransformChanged(const QString& pointCloudName);

private slots:
    void onCellClicked(int row, int col);
    void on_closeButton_clicked();
    void on_refreshButton_clicked();
    void on_exportButton_clicked();

private:
    Ui::GridPopulationDensityDialog *ui;
    const OpenGLWidget::Grid2D& grid;
    QString pointCloudName;
    OpenGLWidget* openGLWidget;
    std::vector<std::vector<int>> cellPointCounts;
    QGridLayout *gridLayout;
    QVector<QVector<QPushButton*>> cellButtons;
    int maxPointCount;

    void setupGridLayout();
    void calculatePointCounts();
    void updateGridInfo();
    QString getCellStyleSheet(int pointCount);
    int countPointsInCell(int row, int col);
    void updateGridVisualization();
};

#endif // GRIDPOPULATIONDENSITYDIALOG_H