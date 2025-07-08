#include "segmentation.h"
#include "ui_segmentation.h"
#include "openglwidget.h"
#include <QDebug>

SegmentationDialog::SegmentationDialog(OpenGLWidget* openGLWidget, QWidget *parent)
    : QDialog(parent)
    , ui(new Ui::SegmentationDialog)
    , openGLWidget(openGLWidget)
    , isInitializing(true)  // Add flag to track initialization
{
    ui->setupUi(this);

    if (!openGLWidget) {
        qDebug() << "SegmentationDialog: openGLWidget is null!";
        return;
    }

    // Apply dark theme stylesheet for consistency
    setStyleSheet(
        "QDialog { background-color: #2E2E2E; color: #FFFFFF; }"
        "QTableWidget { background-color: #3C3C3C; color: #FFFFFF; gridline-color: #555555; }"
        "QTableWidget::item { padding: 5px; }"
        "QHeaderView::section { background-color: #3C3C3C; color: #FFFFFF; padding: 4px; border: 1px solid #555555; }"
        "QPushButton { background-color: #4A4A4A; color: #FFFFFF; border: 1px solid #555555; padding: 5px; }"
        "QPushButton:hover { background-color: #5A5A5A; }"
        "QPushButton:pressed { background-color: #3A3A3A; }"
    );

    // Fix the label text to be accurate
    ui->labelInfo->setText("This dialog shows the grid cells that contain object parts. Check to show, uncheck to hide.");

    populateTable();
    isInitializing = false;  // Initialization complete

    connect(ui->tableWidget, &QTableWidget::itemChanged, this, &SegmentationDialog::on_tableWidget_itemChanged);
    connect(ui->clearSelections, &QPushButton::clicked, this, &SegmentationDialog::on_clearSelections_clicked);
}

SegmentationDialog::~SegmentationDialog()
{
    delete ui;
}

void SegmentationDialog::on_buttonBox_accepted()
{
    accept();
}

void SegmentationDialog::on_buttonBox_rejected()
{
    reject();
}

void SegmentationDialog::populateTable()
{
    if (!ui->tableWidget || !openGLWidget) {
        qDebug() << "populateTable: Invalid tableWidget or openGLWidget";
        return;
    }

    // Disconnect to prevent signals during population
    disconnect(ui->tableWidget, &QTableWidget::itemChanged, this, &SegmentationDialog::on_tableWidget_itemChanged);

    ui->tableWidget->setRowCount(0);
    ui->tableWidget->setColumnCount(4);
    ui->tableWidget->setHorizontalHeaderLabels({"Select", "Grid Name", "Row", "Column"});

    const auto& grids = openGLWidget->get2DGrids();
    qDebug() << "populateTable: Number of 2D grids:" << grids.size();

    // Track the sections we need to make visible
    QList<QPair<QString, QPair<int, int>>> sectionsToActivate;

    for (const auto& grid : grids) {
        if (!grid.visible) {
            qDebug() << "Skipping invisible grid:" << grid.name;
            continue;
        }

        qDebug() << "Processing grid:" << grid.name << "Rows:" << grid.rows << "Cols:" << grid.columns;

        for (int row = 0; row < grid.rows; ++row) {
            for (int col = 0; col < grid.columns; ++col) {
                bool containsObject = false;
                QString sectionName = QString("%1_%2_%3").arg(grid.name).arg(row).arg(col);
                
                // Always set section visible to true as we want all checkboxes selected by default
                bool sectionVisible = true;

                // Check if a point cloud section exists for this cell
                const auto& pointClouds = openGLWidget->getPointClouds();
                for (const auto& pc : pointClouds) {
                    if (pc.name == sectionName) {
                        // We'll set it to true later, but keep track of current state
                        containsObject = true;
                        break;
                    }
                }

                // If no section exists, check for points in this cell
                if (!containsObject) {
                    float cellXMin = grid.min.x() + col * grid.cellSize;
                    float cellXMax = cellXMin + grid.cellSize;
                    float cellZMin = grid.min.z() + row * grid.cellSize;
                    float cellZMax = cellZMin + grid.cellSize;

                    for (const auto& pc : pointClouds) {
                        if (!pc.visible || !pc.parentName.isEmpty()) {
                            continue;
                        }

                        for (const auto& point : pc.points) {
                            float x = point.x();
                            float z = point.z();
                            if (x >= cellXMin && x < cellXMax && z >= cellZMin && z < cellZMax) {
                                containsObject = true;
                                break;
                            }
                        }
                        if (containsObject) {
                            break;
                        }
                    }
                }

                if (containsObject) {
                    int rowCount = ui->tableWidget->rowCount();
                    ui->tableWidget->insertRow(rowCount);

                    QTableWidgetItem *checkItem = new QTableWidgetItem();
                    // Set all checkboxes to checked by default
                    checkItem->setCheckState(Qt::Checked);
                    // Make sure the checkbox is clickable and not editable
                    checkItem->setFlags(Qt::ItemIsEnabled | Qt::ItemIsUserCheckable | Qt::ItemIsSelectable);

                    ui->tableWidget->setItem(rowCount, 0, checkItem);

                    // Create non-editable items for the other columns
                    QTableWidgetItem *gridNameItem = new QTableWidgetItem(grid.name);
                    gridNameItem->setFlags(Qt::ItemIsEnabled | Qt::ItemIsSelectable);
                    ui->tableWidget->setItem(rowCount, 1, gridNameItem);

                    QTableWidgetItem *rowItem = new QTableWidgetItem(QString::number(row));
                    rowItem->setFlags(Qt::ItemIsEnabled | Qt::ItemIsSelectable);
                    ui->tableWidget->setItem(rowCount, 2, rowItem);

                    QTableWidgetItem *colItem = new QTableWidgetItem(QString::number(col));
                    colItem->setFlags(Qt::ItemIsEnabled | Qt::ItemIsSelectable);
                    ui->tableWidget->setItem(rowCount, 3, colItem);

                    // Add this section to our list to make visible
                    sectionsToActivate.append(qMakePair(grid.name, qMakePair(row, col)));

                    qDebug() << "Added table row:" << rowCount << "Grid:" << grid.name
                             << "Row:" << row << "Col:" << col << "Visible:" << sectionVisible
                             << "CheckState: Checked";
                }
            }
        }
    }

    // Make the table widget update its visual appearance
    ui->tableWidget->update();

    // Now actually make all sections visible in the model
    for (const auto& section : sectionsToActivate) {
        openGLWidget->setGridCellVisibility(section.first, section.second.first, section.second.second, true);
    }

    // Make sure the model is updated
    updateScene();

    // Reconnect signal
    connect(ui->tableWidget, &QTableWidget::itemChanged, this, &SegmentationDialog::on_tableWidget_itemChanged);
}

void SegmentationDialog::on_tableWidget_itemChanged(QTableWidgetItem *item)
{
    // Skip during initialization to prevent unwanted triggers
    if (isInitializing || !item || !ui->tableWidget || !openGLWidget) {
        qDebug() << "on_tableWidget_itemChanged: Invalid item, tableWidget, or openGLWidget, or during initialization";
        return;
    }

    if (item->column() == 0) {
        // The checkbox column
        bool checked = (item->checkState() == Qt::Checked);
        int row = item->row();
        
        QTableWidgetItem *gridItem = ui->tableWidget->item(row, 1);
        QTableWidgetItem *rowItem = ui->tableWidget->item(row, 2);
        QTableWidgetItem *colItem = ui->tableWidget->item(row, 3);

        if (!gridItem || !rowItem || !colItem) {
            qDebug() << "on_tableWidget_itemChanged: Invalid table items at row" << row;
            return;
        }

        QString gridName = gridItem->text();
        bool rowOk, colOk;
        int gridRow = rowItem->text().toInt(&rowOk);
        int gridCol = colItem->text().toInt(&colOk);

        if (!rowOk || !colOk) {
            qDebug() << "on_tableWidget_itemChanged: Invalid row or column value at row" << row;
            return;
        }

        // Explicitly block signals during the visibility change to prevent recursion
        QSignalBlocker blocker(ui->tableWidget);
        
        // Checked means visible, Unchecked means hidden
        bool visible = checked;
        
        qDebug() << "Setting visibility for grid cell:" << gridName << gridRow << gridCol 
                 << "Checked:" << checked << "Setting Visible:" << visible;
        
        // Update the model immediately
        openGLWidget->setGridCellVisibility(gridName, gridRow, gridCol, visible);
        
        // Make sure the checkbox state reflects the intended visibility state
        item->setCheckState(visible ? Qt::Checked : Qt::Unchecked);
        
        // End blocking signals
        blocker.unblock();

        // Force a scene update
        updateScene();
    }
}

void SegmentationDialog::on_clearSelections_clicked()
{
    if (!ui->tableWidget || !openGLWidget) {
        qDebug() << "on_clearSelections_clicked: Invalid tableWidget or openGLWidget";
        return;
    }

    QString gridName;
    auto selectedItems = ui->tableWidget->selectedItems();
    if (!selectedItems.isEmpty()) {
        int row = selectedItems.first()->row();
        QTableWidgetItem *gridItem = ui->tableWidget->item(row, 1);
        if (gridItem) {
            gridName = gridItem->text();
        }
    }

    if (gridName.isEmpty() && ui->tableWidget->rowCount() > 0) {
        QTableWidgetItem *gridItem = ui->tableWidget->item(0, 1);
        if (gridItem) {
            gridName = gridItem->text();
        }
    }

    if (gridName.isEmpty()) {
        qDebug() << "on_clearSelections_clicked: No grid selected or available";
        return;
    }

    clearSelections(gridName);
    populateTable();
    updateScene();
}

void SegmentationDialog::clearSelections(const QString& gridName)
{
    if (!openGLWidget) {
        qDebug() << "clearSelections: Invalid openGLWidget";
        return;
    }

    openGLWidget->clearGridCellSelections(gridName);
    updateScene();
}

void SegmentationDialog::updateScene()
{
    if (openGLWidget) {
        openGLWidget->update();
    }
}