#include "gridvisibilitydialog.h"
#include "ui_gridvisibilitydialog.h"
#include <QVBoxLayout>
#include <QCheckBox>

GridVisibilityDialog::GridVisibilityDialog(OpenGLWidget* openGLWidget, const std::vector<OpenGLWidget::Grid2D>& grids2D, QWidget *parent) :
    QDialog(parent),
    ui(new Ui::GridVisibilityDialog),
    m_grids2D(grids2D),
    m_openGLWidget(openGLWidget)
{
    ui->setupUi(this);

    setWindowTitle(tr("Grid Visibility"));

    setStyleSheet(
        "QDialog {"
        "    background-color: #2E2E2E;"
        "    color: #FFFFFF;"
        "}"
        "QLabel {"
        "    color: #FFFFFF;"
        "    font-size: 12px;"
        "}"
        "QCheckBox {"
        "    color: #FFFFFF;"
        "    background-color: #3C3C3C;"
        "    border: 1px solid #555555;"
        "    padding: 4px;"
        "    border-radius: 3px;"
        "}"
        "QCheckBox::indicator {"
        "    width: 16px;"
        "    height: 16px;"
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

    QVBoxLayout* gridsLayout = qobject_cast<QVBoxLayout*>(ui->scrollAreaWidgetContents->layout());
    if (!gridsLayout) return;

    for (const auto& grid : m_grids2D) {
        QCheckBox* checkBox = new QCheckBox(grid.name, ui->scrollAreaWidgetContents);
        checkBox->setChecked(grid.visible);
        checkBox->setObjectName(grid.name);
        gridsLayout->addWidget(checkBox);
        m_checkBoxes.push_back(checkBox);
        connect(checkBox, &QCheckBox::stateChanged, this, &GridVisibilityDialog::on_checkBox_stateChanged);
    }

    gridsLayout->addStretch();

    // Connect gridVisibilityChanged signal to OpenGLWidget's set2DGridVisibility slot
    if (m_openGLWidget) {
        connect(this, &GridVisibilityDialog::gridVisibilityChanged,
                m_openGLWidget, &OpenGLWidget::set2DGridVisibility);
    }
}

GridVisibilityDialog::~GridVisibilityDialog()
{
    delete ui;
}

void GridVisibilityDialog::on_buttonBox_accepted()
{
    for (size_t i = 0; i < m_checkBoxes.size(); ++i) {
        bool visible = m_checkBoxes[i]->isChecked();
        if (visible != m_grids2D[i].visible) {
            emit gridVisibilityChanged(m_grids2D[i].name, visible);
        }
    }
    accept();
}

void GridVisibilityDialog::on_buttonBox_rejected()
{
    reject();
}

void GridVisibilityDialog::on_checkBox_stateChanged(int state)
{
    QCheckBox* checkBox = qobject_cast<QCheckBox*>(sender());
    if (!checkBox) return;
    QString name = checkBox->objectName();
    bool visible = (state == Qt::Checked);
    emit gridVisibilityChanged(name, visible);

    // Find the grid to get its height
    float currentHeight = 0.0f;
    for (const auto& grid : m_grids2D) {
        if (grid.name == name) {
            currentHeight = grid.height;
            break;
        }
    }

    // Sort grids by height to find the next visible grid above
    std::vector<OpenGLWidget::Grid2D> sortedGrids = m_grids2D;
    std::sort(sortedGrids.begin(), sortedGrids.end(),
              [](const OpenGLWidget::Grid2D& a, const OpenGLWidget::Grid2D& b) { return a.height < b.height; });

    float upperHeight = std::numeric_limits<float>::max();
    for (const auto& grid : sortedGrids) {
        if (grid.height > currentHeight && m_openGLWidget->is2DGridVisible(grid.name)) {
            upperHeight = grid.height;
            break;
        }
    }

    // If the grid visibility changes, update object visibility
    if (visible) {
        // Grid is being made visible - restore all objects and then re-split as needed
        m_openGLWidget->updateObjectVisibilityBetweenGrids();
    } else {
        // Grid is being hidden - split objects and hide parts between this grid and next visible grid
        for (const auto& pc : m_openGLWidget->getPointClouds()) {
            if (pc.parentName.isEmpty()) { // Only process original point clouds
                m_openGLWidget->splitPointCloudByHeight(pc.name, currentHeight, upperHeight, true);
            }
        }
        
        for (const auto& model : m_openGLWidget->getModels()) {
            if (model.parentName.isEmpty()) { // Only process original models
                m_openGLWidget->splitModelByHeight(model.name, currentHeight, upperHeight, true);
            }
        }
    }
}
