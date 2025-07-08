#include "gridpopulationdensitydialog.h"
#include "ui_gridpopulationdensitydialog.h"
#include <QMessageBox>
#include <QScrollArea>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QApplication>
#include <QScreen>
#include <QFileDialog>
#include <QTextStream>

GridPopulationDensityDialog::GridPopulationDensityDialog(const OpenGLWidget::Grid2D& grid, 
                                                       const QString& pointCloudName, 
                                                       OpenGLWidget* openGLWidget,
                                                       QWidget *parent)
    : QDialog(parent), 
      ui(new Ui::GridPopulationDensityDialog), 
      grid(grid), 
      pointCloudName(pointCloudName), 
      openGLWidget(openGLWidget),
      maxPointCount(0)
{
    ui->setupUi(this);
    
    QScreen *screen = QApplication::primaryScreen();
    QRect screenGeometry = screen->geometry();
    int width = qMin(800, static_cast<int>(screenGeometry.width() * 0.8));
    int height = qMin(600, static_cast<int>(screenGeometry.height() * 0.8));
    resize(width, height);

    cellPointCounts.resize(grid.rows, std::vector<int>(grid.columns, 0));

    connect(openGLWidget, &OpenGLWidget::pointCloudTransformChanged, 
            this, &GridPopulationDensityDialog::onPointCloudTransformChanged);
    connect(ui->closeButton, &QPushButton::clicked, this, &GridPopulationDensityDialog::on_closeButton_clicked);
    connect(ui->refreshButton, &QPushButton::clicked, this, &GridPopulationDensityDialog::on_refreshButton_clicked);
    connect(ui->exportButton, &QPushButton::clicked, this, &GridPopulationDensityDialog::on_exportButton_clicked);

    setStatusMessage(QString("Calculating point counts for point cloud '%1'...").arg(pointCloudName));
    calculatePointCounts();

    maxPointCount = 0;
    for (const auto& row : cellPointCounts) {
        for (int count : row) {
            maxPointCount = qMax(maxPointCount, count);
        }
    }

    updateGridInfo();
    setStatusMessage("Rendering grid visualization...");
    setupGridLayout();
    updateLegend();

    setStatusMessage(QString("Grid visualization complete: %1 rows, %2 columns")
                     .arg(grid.rows).arg(grid.columns));
}

GridPopulationDensityDialog::~GridPopulationDensityDialog()
{
    delete ui;
}

void GridPopulationDensityDialog::calculatePointCounts()
{
    cellPointCounts.clear();
    cellPointCounts.resize(grid.rows, std::vector<int>(grid.columns, 0));

    for (const auto& pc : openGLWidget->getPointClouds()) {
        if (pc.name == pointCloudName) {
            for (const auto& point : pc.points) {
                QVector3D transformedPoint = openGLWidget->applyTransformations(point, pc.offset, pc.rotation, pc.scale);
                float x = transformedPoint.x();
                float z = transformedPoint.z();
                int row = static_cast<int>((x - grid.min.x()) / grid.cellSize);
                int col = static_cast<int>((z - grid.min.z()) / grid.cellSize);
                if (row >= 0 && row < grid.rows && col >= 0 && col < grid.columns) {
                    cellPointCounts[row][col]++;
                }
            }
            break;
        }
    }
}

void GridPopulationDensityDialog::updateGridInfo()
{
    ui->rowsLabel->setText(QString::number(grid.rows));
    ui->columnsLabel->setText(QString::number(grid.columns));
    ui->cellSizeLabel->setText(QString::number(grid.cellSize, 'f', 2));
}

void GridPopulationDensityDialog::setupGridLayout()
{
    gridLayout = new QGridLayout(ui->gridWidget);
    gridLayout->setSpacing(1);
    gridLayout->setContentsMargins(5, 5, 5, 5);

    int maxButtonSize = qMin(30, qMin(500 / grid.columns, 400 / grid.rows));
    int buttonSize = qMax(8, maxButtonSize);

    cellButtons.resize(grid.rows);
    for (int row = 0; row < grid.rows; ++row) {
        cellButtons[row].resize(grid.columns);
        for (int col = 0; col < grid.columns; ++col) {
            QPushButton *button = new QPushButton();
            button->setFixedSize(buttonSize, buttonSize);
            
            QString styleSheet = getCellStyleSheet(cellPointCounts[row][col]);
            button->setStyleSheet(styleSheet);
            
            button->setProperty("row", row);
            button->setProperty("col", col);
            button->setProperty("count", cellPointCounts[row][col]);

            button->setToolTip(QString("Cell [%1,%2]: %3 points")
                              .arg(row).arg(col).arg(cellPointCounts[row][col]));

            connect(button, &QPushButton::clicked, this, [this, row, col]() {
                onCellClicked(row, col);
            });

            gridLayout->addWidget(button, row, col);
            cellButtons[row][col] = button;
        }
    }

    int minWidth = grid.columns * (buttonSize + 1) + 10;
    int minHeight = grid.rows * (buttonSize + 1) + 10;
    ui->gridWidget->setMinimumSize(minWidth, minHeight);
}

QString GridPopulationDensityDialog::getCellStyleSheet(int pointCount)
{
    QString baseStyle = "QPushButton { border: 1px solid #3A3A3A; border-radius: 1px; }";
    
    if (pointCount == 0) {
        return baseStyle + "QPushButton { background-color: #404040; }";
    } else if (maxPointCount <= 100) {
        if (pointCount <= maxPointCount * 0.2) {
            return baseStyle + "QPushButton { background-color: #90EE90; }";
        } else if (pointCount <= maxPointCount * 0.5) {
            return baseStyle + "QPushButton { background-color: #FFD700; }";
        } else if (pointCount <= maxPointCount * 0.8) {
            return baseStyle + "QPushButton { background-color: #FFA500; }";
        } else {
            return baseStyle + "QPushButton { background-color: #FF4444; }";
        }
    } else if (maxPointCount <= 1000) {
        if (pointCount <= maxPointCount * 0.1) {
            return baseStyle + "QPushButton { background-color: #90EE90; }";
        } else if (pointCount <= maxPointCount * 0.3) {
            return baseStyle + "QPushButton { background-color: #FFD700; }";
        } else if (pointCount <= maxPointCount * 0.6) {
            return baseStyle + "QPushButton { background-color: #FFA500; }";
        } else if (pointCount <= maxPointCount * 0.8) {
            return baseStyle + "QPushButton { background-color: #FF6B6B; }";
        } else {
            return baseStyle + "QPushButton { background-color: #CC0000; }";
        }
    } else {
        if (pointCount <= maxPointCount * 0.05) {
            return baseStyle + "QPushButton { background-color: #90EE90; }";
        } else if (pointCount <= maxPointCount * 0.15) {
            return baseStyle + "QPushButton { background-color: #ADFF2F; }";
        } else if (pointCount <= maxPointCount * 0.35) {
            return baseStyle + "QPushButton { background-color: #FFD700; }";
        } else if (pointCount <= maxPointCount * 0.55) {
            return baseStyle + "QPushButton { background-color: #FFA500; }";
        } else if (pointCount <= maxPointCount * 0.75) {
            return baseStyle + "QPushButton { background-color: #FF6B6B; }";
        } else if (pointCount <= maxPointCount * 0.9) {
            return baseStyle + "QPushButton { background-color: #FF4444; }";
        } else {
            return baseStyle + "QPushButton { background-color: #8B0000; }";
        }
    }
}

int GridPopulationDensityDialog::countPointsInCell(int row, int col)
{
    return cellPointCounts[row][col];
}

void GridPopulationDensityDialog::setStatusMessage(const QString& message)
{
    ui->statusLabel->setText(message);
    QApplication::processEvents();
}

void GridPopulationDensityDialog::onCellClicked(int row, int col)
{
    int pointCount = countPointsInCell(row, col);
    
    QDialog *detailDialog = new QDialog(this);
    detailDialog->setWindowTitle("Cell Details");
    detailDialog->setFixedSize(420, 280);
    detailDialog->setStyleSheet(this->styleSheet());
    
    QVBoxLayout *layout = new QVBoxLayout(detailDialog);
    layout->setContentsMargins(15, 15, 15, 15);
    layout->setSpacing(10);
    
    QLabel *titleLabel = new QLabel("Cell Information");
    titleLabel->setStyleSheet("font-size: 16px; font-weight: bold; color: #FFFFFF; margin-bottom: 15px;");
    titleLabel->setAlignment(Qt::AlignCenter);
    layout->addWidget(titleLabel);
    
    QLabel *detailsLabel = new QLabel();
    detailsLabel->setStyleSheet("font-size: 13px; color: #CCCCCC; line-height: 1.6; padding: 10px;");
    detailsLabel->setWordWrap(true);
    
    QString densityCategory;
    if (pointCount == 0) {
        densityCategory = "Empty";
    } else if (maxPointCount <= 100) {
        if (pointCount <= maxPointCount * 0.2) densityCategory = "Very Low";
        else if (pointCount <= maxPointCount * 0.5) densityCategory = "Low";
        else if (pointCount <= maxPointCount * 0.8) densityCategory = "Medium";
        else densityCategory = "High";
    } else if (maxPointCount <= 1000) {
        if (pointCount <= maxPointCount * 0.1) densityCategory = "Very Low";
        else if (pointCount <= maxPointCount * 0.3) densityCategory = "Low";
        else if (pointCount <= maxPointCount * 0.6) densityCategory = "Medium";
        else if (pointCount <= maxPointCount * 0.8) densityCategory = "High";
        else densityCategory = "Very High";
    } else {
        if (pointCount <= maxPointCount * 0.05) densityCategory = "Very Low";
        else if (pointCount <= maxPointCount * 0.15) densityCategory = "Low";
        else if (pointCount <= maxPointCount * 0.35) densityCategory = "Medium-Low";
        else if (pointCount <= maxPointCount * 0.55) densityCategory = "Medium";
        else if (pointCount <= maxPointCount * 0.75) densityCategory = "Medium-High";
        else if (pointCount <= maxPointCount * 0.9) densityCategory = "High";
        else densityCategory = "Very High";
    }
    
    QString details = QString(
        "<b>Grid Position:</b><br/>"
        "&nbsp;&nbsp;• Row: %1<br/>"
        "&nbsp;&nbsp;• Column: %2<br/><br/>"
        "<b>Point Count:</b> %3<br/><br/>"
        "<b>Cell Coordinates:</b><br/>"
        "&nbsp;&nbsp;• X Range: %4 to %5<br/>"
        "&nbsp;&nbsp;• Z Range: %6 to %7<br/><br/>"
        "<b>Density Category:</b> %8<br/>"
        "<b>Relative Density:</b> %9% of maximum"
    ).arg(row)
     .arg(col)
     .arg(pointCount)
     .arg(grid.min.x() + row * grid.cellSize, 0, 'f', 3)
     .arg(grid.min.x() + (row + 1) * grid.cellSize, 0, 'f', 3)
     .arg(grid.min.z() + col * grid.cellSize, 0, 'f', 3)
     .arg(grid.min.z() + (col + 1) * grid.cellSize, 0, 'f', 3)
     .arg(densityCategory)
     .arg(maxPointCount > 0 ? QString::number((pointCount * 100.0) / maxPointCount, 'f', 1) : "0.0");
    
    detailsLabel->setText(details);
    layout->addWidget(detailsLabel);
    
    QPushButton *closeBtn = new QPushButton("Close");
    closeBtn->setStyleSheet("QPushButton { background-color: #007ACC; border: 1px solid #005F9E; padding: 10px 24px; border-radius: 4px; font-size: 13px; min-width: 80px; }");
    connect(closeBtn, &QPushButton::clicked, detailDialog, &QDialog::accept);
    
    QHBoxLayout *buttonLayout = new QHBoxLayout();
    buttonLayout->addStretch();
    buttonLayout->addWidget(closeBtn);
    buttonLayout->addStretch();
    layout->addLayout(buttonLayout);
    
    detailDialog->exec();
    delete detailDialog;
}

void GridPopulationDensityDialog::on_closeButton_clicked()
{
    reject();
}

void GridPopulationDensityDialog::updateLegend()
{
    if (maxPointCount > 0) {
        if (maxPointCount <= 100) {
            int lowThreshold = static_cast<int>(maxPointCount * 0.2);
            int mediumLowThreshold = static_cast<int>(maxPointCount * 0.5);
            int mediumHighThreshold = static_cast<int>(maxPointCount * 0.8);
            
            ui->lowLabel->setText(QString("Very Low (1-%1)").arg(lowThreshold));
            ui->mediumLabel->setText(QString("Low (%1-%2)").arg(lowThreshold + 1).arg(mediumLowThreshold));
            ui->highLabel->setText(QString("Med (%1-%2)").arg(mediumLowThreshold + 1).arg(mediumHighThreshold));
        } else if (maxPointCount <= 1000) {
            int veryLowThreshold = static_cast<int>(maxPointCount * 0.1);
            int lowThreshold = static_cast<int>(maxPointCount * 0.3);
            int mediumThreshold = static_cast<int>(maxPointCount * 0.6);
            int highThreshold = static_cast<int>(maxPointCount * 0.8);
            
            ui->lowLabel->setText(QString("Low (1-%1)").arg(veryLowThreshold));
            ui->mediumLabel->setText(QString("Med (%1-%2)").arg(lowThreshold + 1).arg(mediumThreshold));
            ui->highLabel->setText(QString("High (%1+)").arg(highThreshold + 1));
        } else {
            int veryLowThreshold = static_cast<int>(maxPointCount * 0.05);
            int lowThreshold = static_cast<int>(maxPointCount * 0.15);
            int mediumThreshold = static_cast<int>(maxPointCount * 0.55);
            int highThreshold = static_cast<int>(maxPointCount * 0.75);
            
            ui->lowLabel->setText(QString("Low (1-%1)").arg(veryLowThreshold));
            ui->mediumLabel->setText(QString("Med (%1-%2)").arg(lowThreshold + 1).arg(mediumThreshold));
            ui->highLabel->setText(QString("High (%1+)").arg(highThreshold + 1));
        }
    } else {
        ui->lowLabel->setText("Low");
        ui->mediumLabel->setText("Medium");
        ui->highLabel->setText("High");
    }
}

void GridPopulationDensityDialog::onPointCloudTransformChanged(const QString& name)
{
    if (name == pointCloudName) {
        setStatusMessage(QString("Updating grid visualization for '%1'...").arg(pointCloudName));
        calculatePointCounts();
        
        maxPointCount = 0;
        for (const auto& row : cellPointCounts) {
            for (int count : row) {
                maxPointCount = qMax(maxPointCount, count);
            }
        }
        
        updateGridVisualization();
        updateLegend();
        
        setStatusMessage(QString("Grid visualization updated for '%1'").arg(pointCloudName));
    }
}

void GridPopulationDensityDialog::updateGridVisualization()
{
    for (int row = 0; row < grid.rows; ++row) {
        for (int col = 0; col < grid.columns; ++col) {
            QPushButton* button = cellButtons[row][col];
            QString styleSheet = getCellStyleSheet(cellPointCounts[row][col]);
            button->setStyleSheet(styleSheet);
            button->setProperty("count", cellPointCounts[row][col]);
            button->setToolTip(QString("Cell [%1,%2]: %3 points")
                              .arg(row).arg(col).arg(cellPointCounts[row][col]));
        }
    }
}

void GridPopulationDensityDialog::on_refreshButton_clicked()
{
    setStatusMessage(QString("Refreshing grid visualization for '%1'...").arg(pointCloudName));
    calculatePointCounts();
    
    maxPointCount = 0;
    for (const auto& row : cellPointCounts) {
        for (int count : row) {
            maxPointCount = qMax(maxPointCount, count);
        }
    }
    
    updateGridVisualization();
    updateLegend();
    updateGridInfo();
    
    setStatusMessage(QString("Grid visualization refreshed: %1 rows, %2 columns")
                     .arg(grid.rows).arg(grid.columns));
}

void GridPopulationDensityDialog::on_exportButton_clicked()
{
    QString fileName = QFileDialog::getSaveFileName(this, tr("Export Grid Data"), "", tr("CSV Files (*.csv)"));
    if (fileName.isEmpty()) return;

    QFile file(fileName);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
        QMessageBox::warning(this, tr("Export Error"), tr("Cannot open file for writing."));
        return;
    }

    QTextStream out(&file);
    out << "Row,Column,Point Count,X Min,X Max,Z Min,Z Max,Density Category\n";
    
    for (int row = 0; row < grid.rows; ++row) {
        for (int col = 0; col < grid.columns; ++col) {
            int pointCount = cellPointCounts[row][col];
            QString densityCategory;
            if (pointCount == 0) {
                densityCategory = "Empty";
            } else if (maxPointCount <= 100) {
                if (pointCount <= maxPointCount * 0.2) densityCategory = "Very Low";
                else if (pointCount <= maxPointCount * 0.5) densityCategory = "Low";
                else if (pointCount <= maxPointCount * 0.8) densityCategory = "Medium";
                else densityCategory = "High";
            } else if (maxPointCount <= 1000) {
                if (pointCount <= maxPointCount * 0.1) densityCategory = "Very Low";
                else if (pointCount <= maxPointCount * 0.3) densityCategory = "Low";
                else if (pointCount <= maxPointCount * 0.6) densityCategory = "Medium";
                else if (pointCount <= maxPointCount * 0.8) densityCategory = "High";
                else densityCategory = "Very High";
            } else {
                if (pointCount <= maxPointCount * 0.05) densityCategory = "Very Low";
                else if (pointCount <= maxPointCount * 0.15) densityCategory = "Low";
                else if (pointCount <= maxPointCount * 0.35) densityCategory = "Medium-Low";
                else if (pointCount <= maxPointCount * 0.55) densityCategory = "Medium";
                else if (pointCount <= maxPointCount * 0.75) densityCategory = "Medium-High";
                else if (pointCount <= maxPointCount * 0.9) densityCategory = "High";
                else densityCategory = "Very High";
            }

            out << row << ","
                << col << ","
                << pointCount << ","
                << QString::number(grid.min.x() + row * grid.cellSize, 'f', 3) << ","
                << QString::number(grid.min.x() + (row + 1) * grid.cellSize, 'f', 3) << ","
                << QString::number(grid.min.z() + col * grid.cellSize, 'f', 3) << ","
                << QString::number(grid.min.z() + (col + 1) * grid.cellSize, 'f', 3) << ","
                << densityCategory << "\n";
        }
    }

    file.close();
    setStatusMessage(tr("Grid data exported successfully to %1").arg(fileName));
}