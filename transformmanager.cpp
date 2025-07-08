#include "transformmanager.h"
#include "openglwidget.h"
#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QDebug>
#include <QDoubleSpinBox>
#include <QCheckBox>
#include <QPushButton>
#include <QTreeWidgetItem>
#include <QFileInfo> // Added missing include
#include<QRegularExpression>


TransformManager::TransformManager(OpenGLWidget* openGLWidget, MainWindow* mainWindow, QObject* parent)
    : QObject(parent),
    m_openGLWidget(openGLWidget),
    m_mainWindow(mainWindow),
    m_selectedPointCloud(""),
    m_selectedModel(""),
    m_selectedModelName(""),
    m_selectedNodeIndex(-1),
    m_lockScaleProportions(true),
    m_updatingUI(false)
{
    // Ensure we have valid pointers
    if (!m_openGLWidget || !m_mainWindow) {
        qCritical() << "TransformManager initialized with null pointer(s)!";
    }
    // Connect to OpenGLWidget signals
    if (m_openGLWidget) {
        connect(m_openGLWidget, &OpenGLWidget::pointCloudSelected,
                this, &TransformManager::onPointCloudSelected);
        connect(m_openGLWidget, &OpenGLWidget::pointCloudTransformChanged,
                this, &TransformManager::onPointCloudTransformChanged);
    }
}
// Add these methods to the TransformManager class
void TransformManager::onPointCloudSelected(const QString& filePath)
{

    m_selectedPointCloud = filePath;

    updateUIFromSelection();
}

void TransformManager::onPointCloudTransformChanged(const QString& filePath)
{
    if (m_selectedPointCloud == filePath) {
        updateUIFromSelection();
    }
}
// 2. Modify setupConnections to handle OpenGLWidget signals for all views
void TransformManager::setupConnections()
{
    // Get the UI from the main window
    Ui::MainWindow* ui = m_mainWindow->ui;

    // Connect transform UI controls to slots
    if (ui->positionX) {
        connect(ui->positionX, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
                this, &TransformManager::setPositionX);
    }

    if (ui->positionY) {
        connect(ui->positionY, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
                this, &TransformManager::setPositionY);
    }

    if (ui->positionZ) {
        connect(ui->positionZ, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
                this, &TransformManager::setPositionZ);
    }

    if (ui->rotationX) {
        connect(ui->rotationX, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
                this, &TransformManager::setRotationX);
    }

    if (ui->rotationY) {
        connect(ui->rotationY, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
                this, &TransformManager::setRotationY);
    }

    if (ui->rotationZ) {
        connect(ui->rotationZ, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
                this, &TransformManager::setRotationZ);
    }

    if (ui->scaleX) {
        connect(ui->scaleX, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
                this, &TransformManager::setScaleX);
    }

    if (ui->scaleY) {
        connect(ui->scaleY, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
                this, &TransformManager::setScaleY);
    }

    if (ui->scaleZ) {
        connect(ui->scaleZ, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
                this, &TransformManager::setScaleZ);
    }

    // Connect to initial OpenGLWidget
    if (m_openGLWidget) {
        connect(m_openGLWidget, &OpenGLWidget::pointCloudSelected,
                this, &TransformManager::onPointCloudSelected);
        connect(m_openGLWidget, &OpenGLWidget::modelSelected,
                this, &TransformManager::onModelSelected);
        connect(m_openGLWidget, &OpenGLWidget::modelNodeSelected,
                this, &TransformManager::onModelNodeSelected);
        connect(m_openGLWidget, &OpenGLWidget::clicked,
                this, [this]() {
                    // Update the active view widget when clicked
                    m_mainWindow->setActiveViewWidget(m_openGLWidget);
                });

    }

    // Connect lock scale checkbox
    if (ui->lockScaleCheckbox) {
        connect(ui->lockScaleCheckbox, &QCheckBox::toggled,
                this, &TransformManager::onLockScaleCheckboxToggled);

        // Set initial state
        ui->lockScaleCheckbox->setChecked(m_lockScaleProportions);
    }
}
void TransformManager::enableTransformControls()
{
    // Enable all transform controls
    // Use QRegularExpression instead of QRegExp for Qt 6.x
    QList<QDoubleSpinBox*> spinboxes = m_mainWindow->findChildren<QDoubleSpinBox*>();

    // Filter the spinboxes by name
    for (auto spinbox : spinboxes) {
        QString name = spinbox->objectName();
        if (name.contains("position") || name.contains("rotation") || name.contains("scale")) {
            spinbox->setEnabled(true);
        }
    }

    // Enable reset button if it exists
    QPushButton* resetButton = m_mainWindow->findChild<QPushButton*>("resetTransformButton");
    if (resetButton) {
        resetButton->setEnabled(true);
    }

    // Enable lock scale checkbox if it exists
    QCheckBox* lockScaleCheckbox = m_mainWindow->findChild<QCheckBox*>("lockScaleCheckbox");
    if (lockScaleCheckbox) {
        lockScaleCheckbox->setEnabled(true);
    }
}

void TransformManager::disableTransformControls()
{
    // Disable all transform controls
    // Use QRegularExpression instead of QRegExp for Qt 6.x
    QList<QDoubleSpinBox*> spinboxes = m_mainWindow->findChildren<QDoubleSpinBox*>();

    // Filter the spinboxes by name
    for (auto spinbox : spinboxes) {
        QString name = spinbox->objectName();
        if (name.contains("position") || name.contains("rotation") || name.contains("scale")) {
            spinbox->setEnabled(false);
        }
    }

    // Disable reset button if it exists
    QPushButton* resetButton = m_mainWindow->findChild<QPushButton*>("resetTransformButton");
    if (resetButton) {
        resetButton->setEnabled(false);
    }

    // Disable lock scale checkbox if it exists
    QCheckBox* lockScaleCheckbox = m_mainWindow->findChild<QCheckBox*>("lockScaleCheckbox");
    if (lockScaleCheckbox) {
        lockScaleCheckbox->setEnabled(false);
    }

    // Clear selected items
    m_selectedPointCloud.clear();
    m_selectedModel.clear();
    m_selectedModelName.clear();
    m_selectedNodeIndex = -1;
}

void TransformManager::onModelNodeSelected(const QString& modelName, int nodeIndex)
{
    // Clear other selections
    m_selectedPointCloud.clear();
    m_selectedModel.clear();

    // Set the selected model node
    m_selectedModelName = modelName;
    m_selectedNodeIndex = nodeIndex;

    // Update UI with node transform values
    OpenGLWidget* activeView = m_mainWindow->getActiveViewWidget();
    if (!activeView) return;

    // Find the model and node
    for (const auto& model : activeView->getModels()) {
        if (model.name == modelName && nodeIndex >= 0 && nodeIndex < model.nodes.size()) {
            const auto& node = model.nodes[nodeIndex];

            // Update UI with node transform values
            updatePositionUI(node.position);
            updateRotationUI(node.rotation);
            updateScaleUI(node.scale);
            break;
        }
    }
}

void TransformManager::onTreeItemSelectionChanged(QTreeWidgetItem* item)
{
    if (!item) return;

    // Check if this is a file item (top-level item with stored file path)
    if (!item->parent()) {
        QString filePath = item->data(0, Qt::UserRole).toString();
        if (!filePath.isEmpty()) {
            m_selectedPointCloud = filePath;

            // Update UI with the current transformation values
            updateUIFromSelection();

            qDebug() << "Selected point cloud for transformation:" << filePath;
        }
    } else {
        // If it's a child item, select the parent (which is the file)
        QTreeWidgetItem* parentItem = item->parent();
        if (parentItem) {
            // Recursively call this function with the parent item
            onTreeItemSelectionChanged(parentItem);
        }
    }
}

void TransformManager::updatePositionUI(const QVector3D& position)
{
    m_updatingUI = true;

    QDoubleSpinBox* posX = findSpinBox("positionX");
    QDoubleSpinBox* posY = findSpinBox("positionY");
    QDoubleSpinBox* posZ = findSpinBox("positionZ");

    if (posX) posX->setValue(position.x());
    if (posY) posY->setValue(position.y());
    if (posZ) posZ->setValue(position.z());

    m_updatingUI = false;
}

void TransformManager::updateRotationUI(const QVector3D& rotation)
{
    m_updatingUI = true;

    QDoubleSpinBox* rotX = findSpinBox("rotationX");
    QDoubleSpinBox* rotY = findSpinBox("rotationY");
    QDoubleSpinBox* rotZ = findSpinBox("rotationZ");

    if (rotX) rotX->setValue(rotation.x());
    if (rotY) rotY->setValue(rotation.y());
    if (rotZ) rotZ->setValue(rotation.z());

    m_updatingUI = false;
}

void TransformManager::updateScaleUI(const QVector3D& scale)
{
    m_updatingUI = true;

    QDoubleSpinBox* scaleX = findSpinBox("scaleX");
    QDoubleSpinBox* scaleY = findSpinBox("scaleY");
    QDoubleSpinBox* scaleZ = findSpinBox("scaleZ");

    if (scaleX) scaleX->setValue(scale.x());
    if (scaleY) scaleY->setValue(scale.y());
    if (scaleZ) scaleZ->setValue(scale.z());

    m_updatingUI = false;
}

QDoubleSpinBox* TransformManager::findSpinBox(const QString& objectName)
{
    if (!m_mainWindow) return nullptr;

    return m_mainWindow->findChild<QDoubleSpinBox*>(objectName);
}




void TransformManager::resetTransform()
{
    if (m_selectedPointCloud.isEmpty()) return;

    // Reset position
    auto& pointClouds = const_cast<std::map<QString, OpenGLWidget::PointCloudData>&>(
        m_openGLWidget->getPointCloudMap());

    auto it = pointClouds.find(m_selectedPointCloud);
    if (it != pointClouds.end()) {
        // Reset offset (position)
        it->second.offset = QVector3D(0.0f, 0.0f, 0.0f);

        // Reset rotation and scale
        it->second.rotation = QVector3D(0.0f, 0.0f, 0.0f);
        it->second.scale = QVector3D(1.0f, 1.0f, 1.0f);

        // Update UI
        updateUIFromSelection();

        // Trigger a redraw
        m_openGLWidget->update();

        qDebug() << "Reset transformation for" << m_selectedPointCloud;
    }
}

// Add this method to handle model selection
void TransformManager::onModelSelected(const QString& name)
{
    m_selectedPointCloud = ""; // Clear point cloud selection
    m_selectedModel = name;    // Set model selection
    updateUIFromSelection();
}


void TransformManager::onLockScaleCheckboxToggled(bool checked)
{
    m_lockScaleProportions = checked;
    qDebug() << "Scale proportions locked:" << checked;
}

// Update setPositionX to also handle model nodes
void TransformManager::setPositionX(double x)
{
    if (m_updatingUI) return;

    // Get the active view from the main window
    OpenGLWidget* activeView = m_mainWindow->getActiveViewWidget();
    if (!activeView) return;

    // Try to update point cloud if one is selected
    if (!m_selectedPointCloud.isEmpty()) {
        for (auto& pc : activeView->m_pointClouds) {
            if (pc.name == m_selectedPointCloud) {
                // Calculate center
                QVector3D center = pc.boundingBox();

                // The offset is (position - center)
                QVector3D newOffset = pc.offset;
                newOffset.setX(static_cast<float>(x - center.x()));
                pc.offset = newOffset;

                // Trigger a redraw
                activeView->update();
                return;
            }
        }
    }

    // Try to update model if one is selected
    if (!m_selectedModel.isEmpty()) {
        for (auto& model : activeView->m_models) {
            if (model.name == m_selectedModel) {
                QVector3D position = model.position;
                position.setX(static_cast<float>(x));
                model.position = position;

                // Trigger a redraw
                activeView->update();
                return;
            }
        }
    }

    // Try to update model node if one is selected
    if (!m_selectedModelName.isEmpty() && m_selectedNodeIndex >= 0) {
        for (auto& model : activeView->m_models) {
            if (model.name == m_selectedModelName &&
                m_selectedNodeIndex < model.nodes.size()) {

                QVector3D position = model.nodes[m_selectedNodeIndex].position;
                position.setX(static_cast<float>(x));
                model.nodes[m_selectedNodeIndex].position = position;

                // Trigger a redraw
                activeView->update();

                // Emit signal for model node transform changed
                activeView->modelNodeTransformChanged(m_selectedModelName, m_selectedNodeIndex);
                return;
            }
        }
    }
}

// 4. Update setPositionY to work with the active view
void TransformManager::setPositionY(double y)
{
    if (m_updatingUI) return;

    // Get the active view from the main window
    OpenGLWidget* activeView = m_mainWindow->getActiveViewWidget();
    if (!activeView) return;

    // Try to update point cloud if one is selected
    if (!m_selectedPointCloud.isEmpty()) {
        for (auto& pc : activeView->m_pointClouds) {
            if (pc.name == m_selectedPointCloud) {
                // Calculate center
                QVector3D center = pc.boundingBox();

                // The offset is (position - center)
                QVector3D newOffset = pc.offset;
                newOffset.setY(static_cast<float>(y - center.y()));
                pc.offset = newOffset;

                // Trigger a redraw
                activeView->update();
                return;
            }
        }
    }

    // Try to update model if one is selected
    if (!m_selectedModel.isEmpty()) {
        for (auto& model : activeView->m_models) {
            if (model.name == m_selectedModel) {
                QVector3D position = model.position;
                position.setY(static_cast<float>(y));
                model.position = position;

                // Trigger a redraw
                activeView->update();
                return;
            }
        }
    }
}

// 5. Update setPositionZ to work with the active view
void TransformManager::setPositionZ(double z)
{
    if (m_updatingUI) return;

    // Get the active view from the main window
    OpenGLWidget* activeView = m_mainWindow->getActiveViewWidget();
    if (!activeView) return;

    // Try to update point cloud if one is selected
    if (!m_selectedPointCloud.isEmpty()) {
        for (auto& pc : activeView->m_pointClouds) {
            if (pc.name == m_selectedPointCloud) {
                // Calculate center
                QVector3D center = pc.boundingBox();

                // The offset is (position - center)
                QVector3D newOffset = pc.offset;
                newOffset.setZ(static_cast<float>(z - center.z()));
                pc.offset = newOffset;

                // Trigger a redraw
                activeView->update();
                return;
            }
        }
    }

    // Try to update model if one is selected
    if (!m_selectedModel.isEmpty()) {
        for (auto& model : activeView->m_models) {
            if (model.name == m_selectedModel) {
                QVector3D position = model.position;
                position.setZ(static_cast<float>(z));
                model.position = position;

                // Trigger a redraw
                activeView->update();
                return;
            }
        }
    }
}

// 6. Update setRotationX to work with the active view
void TransformManager::setRotationX(double x)
{
    if (m_updatingUI) return;

    // Get the active view from the main window
    OpenGLWidget* activeView = m_mainWindow->getActiveViewWidget();
    if (!activeView) return;

    // Try to update point cloud if one is selected
    if (!m_selectedPointCloud.isEmpty()) {
        for (auto& pc : activeView->m_pointClouds) {
            if (pc.name == m_selectedPointCloud) {
                pc.rotation.setX(static_cast<float>(x));
                activeView->update();
                return;
            }
        }
    }

    // Try to update model if one is selected
    if (!m_selectedModel.isEmpty()) {
        for (auto& model : activeView->m_models) {
            if (model.name == m_selectedModel) {
                model.rotation.setX(static_cast<float>(x));
                activeView->update();
                return;
            }
        }
    }
}

// 7. Update setRotationY to work with the active view
void TransformManager::setRotationY(double y)
{
    if (m_updatingUI) return;

    // Get the active view from the main window
    OpenGLWidget* activeView = m_mainWindow->getActiveViewWidget();
    if (!activeView) return;

    // Try to update point cloud if one is selected
    if (!m_selectedPointCloud.isEmpty()) {
        for (auto& pc : activeView->m_pointClouds) {
            if (pc.name == m_selectedPointCloud) {
                pc.rotation.setY(static_cast<float>(y));
                activeView->update();
                return;
            }
        }
    }

    // Try to update model if one is selected
    if (!m_selectedModel.isEmpty()) {
        for (auto& model : activeView->m_models) {
            if (model.name == m_selectedModel) {
                model.rotation.setY(static_cast<float>(y));
                activeView->update();
                return;
            }
        }
    }
}

// 8. Update setRotationZ to work with the active view
void TransformManager::setRotationZ(double z)
{
    if (m_updatingUI) return;

    // Get the active view from the main window
    OpenGLWidget* activeView = m_mainWindow->getActiveViewWidget();
    if (!activeView) return;

    // Try to update point cloud if one is selected
    if (!m_selectedPointCloud.isEmpty()) {
        for (auto& pc : activeView->m_pointClouds) {
            if (pc.name == m_selectedPointCloud) {
                pc.rotation.setZ(static_cast<float>(z));
                activeView->update();
                return;
            }
        }
    }

    // Try to update model if one is selected
    if (!m_selectedModel.isEmpty()) {
        for (auto& model : activeView->m_models) {
            if (model.name == m_selectedModel) {
                model.rotation.setZ(static_cast<float>(z));
                activeView->update();
                return;
            }
        }
    }
}

// 9. Update setScaleX to work with the active view
void TransformManager::setScaleX(double x)
{
    if (m_updatingUI) return;

    // Get the active view from the main window
    OpenGLWidget* activeView = m_mainWindow->getActiveViewWidget();
    if (!activeView) return;

    // Apply scale to X axis (0)
    applyScale(x, 0, activeView);
}

// 10. Update setScaleY to work with the active view
void TransformManager::setScaleY(double y)
{
    if (m_updatingUI) return;

    // Get the active view from the main window
    OpenGLWidget* activeView = m_mainWindow->getActiveViewWidget();
    if (!activeView) return;

    // Apply scale to Y axis (1)
    applyScale(y, 1, activeView);
}

// 11. Update setScaleZ to work with the active view
void TransformManager::setScaleZ(double z)
{
    if (m_updatingUI) return;

    // Get the active view from the main window
    OpenGLWidget* activeView = m_mainWindow->getActiveViewWidget();
    if (!activeView) return;

    // Apply scale to Z axis (2)
    applyScale(z, 2, activeView);
}

// 12. Update applyScale to work with the active view
void TransformManager::applyScale(double newValue, int axis, OpenGLWidget* activeView)
{
    if (m_updatingUI) return;
    if (!activeView) return;

    // Try to update point cloud if one is selected
    if (!m_selectedPointCloud.isEmpty()) {
        for (auto& pc : activeView->m_pointClouds) {
            if (pc.name == m_selectedPointCloud) {
                QVector3D scale = pc.scale;
                float oldValue = 0.0f;

                // Get old value of the axis being changed
                if (axis == 0) oldValue = scale.x();
                else if (axis == 1) oldValue = scale.y();
                else if (axis == 2) oldValue = scale.z();

                // Calculate ratio for proportional scaling
                float ratio = oldValue > 0 ? static_cast<float>(newValue) / oldValue : 1.0f;

                // Apply the new scale to the specified axis
                if (axis == 0) scale.setX(static_cast<float>(newValue));
                else if (axis == 1) scale.setY(static_cast<float>(newValue));
                else if (axis == 2) scale.setZ(static_cast<float>(newValue));

                // If proportions are locked, scale other axes proportionally
                if (m_lockScaleProportions && oldValue > 0) {
                    // Scale other axes by the same ratio
                    if (axis != 0) scale.setX(scale.x() * ratio);
                    if (axis != 1) scale.setY(scale.y() * ratio);
                    if (axis != 2) scale.setZ(scale.z() * ratio);

                    // Update UI to show the scaled values
                    m_updatingUI = true;
                    QDoubleSpinBox* scaleX = findSpinBox("scaleX");
                    QDoubleSpinBox* scaleY = findSpinBox("scaleY");
                    QDoubleSpinBox* scaleZ = findSpinBox("scaleZ");

                    if (scaleX && axis != 0) scaleX->setValue(scale.x());
                    if (scaleY && axis != 1) scaleY->setValue(scale.y());
                    if (scaleZ && axis != 2) scaleZ->setValue(scale.z());
                    m_updatingUI = false;
                }

                // Apply the new scale to the point cloud
                pc.scale = scale;

                // Trigger a redraw
                activeView->update();
                return;
            }
        }
    }

    // Try to update model if one is selected
    if (!m_selectedModel.isEmpty()) {
        for (auto& model : activeView->m_models) {
            if (model.name == m_selectedModel) {
                QVector3D scale = model.scale;
                float oldValue = 0.0f;

                // Get old value of the axis being changed
                if (axis == 0) oldValue = scale.x();
                else if (axis == 1) oldValue = scale.y();
                else if (axis == 2) oldValue = scale.z();

                // Calculate ratio for proportional scaling
                float ratio = oldValue > 0 ? static_cast<float>(newValue) / oldValue : 1.0f;

                // Apply the new scale to the specified axis
                if (axis == 0) scale.setX(static_cast<float>(newValue));
                else if (axis == 1) scale.setY(static_cast<float>(newValue));
                else if (axis == 2) scale.setZ(static_cast<float>(newValue));

                // If proportions are locked, scale other axes proportionally
                if (m_lockScaleProportions && oldValue > 0) {
                    // Scale other axes by the same ratio
                    if (axis != 0) scale.setX(scale.x() * ratio);
                    if (axis != 1) scale.setY(scale.y() * ratio);
                    if (axis != 2) scale.setZ(scale.z() * ratio);

                    // Update UI to show the scaled values
                    m_updatingUI = true;
                    QDoubleSpinBox* scaleX = findSpinBox("scaleX");
                    QDoubleSpinBox* scaleY = findSpinBox("scaleY");
                    QDoubleSpinBox* scaleZ = findSpinBox("scaleZ");

                    if (scaleX && axis != 0) scaleX->setValue(scale.x());
                    if (scaleY && axis != 1) scaleY->setValue(scale.y());
                    if (scaleZ && axis != 2) scaleZ->setValue(scale.z());
                    m_updatingUI = false;
                }

                // Apply the new scale to the model
                model.scale = scale;

                // Trigger a redraw
                activeView->update();
                return;
            }
        }
    }
}

void TransformManager::updateUIFromSelection()
{
    // Get the active view from main window
    OpenGLWidget* activeView = m_mainWindow->getActiveViewWidget();
    if (!activeView) return;

    if (m_selectedPointCloud.isEmpty() && m_selectedModel.isEmpty() &&
        (m_selectedModelName.isEmpty() || m_selectedNodeIndex < 0)) {
        // No selection, clear or disable UI
        QDoubleSpinBox* posX = findSpinBox("positionX");
        QDoubleSpinBox* posY = findSpinBox("positionY");
        QDoubleSpinBox* posZ = findSpinBox("positionZ");
        QDoubleSpinBox* rotX = findSpinBox("rotationX");
        QDoubleSpinBox* rotY = findSpinBox("rotationY");
        QDoubleSpinBox* rotZ = findSpinBox("rotationZ");
        QDoubleSpinBox* scaleX = findSpinBox("scaleX");
        QDoubleSpinBox* scaleY = findSpinBox("scaleY");
        QDoubleSpinBox* scaleZ = findSpinBox("scaleZ");

        if (posX) posX->setValue(0.0);
        if (posY) posY->setValue(0.0);
        if (posZ) posZ->setValue(0.0);
        if (rotX) rotX->setValue(0.0);
        if (rotY) rotY->setValue(0.0);
        if (rotZ) rotZ->setValue(0.0);
        if (scaleX) scaleX->setValue(1.0);
        if (scaleY) scaleY->setValue(1.0);
        if (scaleZ) scaleZ->setValue(1.0);

        return;
    }

    // Check if a point cloud is selected
    if (!m_selectedPointCloud.isEmpty()) {
        // Find the selected point cloud in the active view
        bool found = false;
        for (const auto& pc : activeView->m_pointClouds) {
            if (pc.name == m_selectedPointCloud) {
                found = true;

                // Calculate center for position
                QVector3D center = pc.boundingBox();
                QVector3D position = center + pc.offset;

                // Set position
                m_updatingUI = true;
                updatePositionUI(position);

                // Set rotation
                updateRotationUI(pc.rotation);

                // Set scale
                updateScaleUI(pc.scale);
                m_updatingUI = false;

                break;
            }
        }

        // If the point cloud wasn't found in this view, clear the selection
        if (!found) {
            m_selectedPointCloud = "";
            updateUIFromSelection(); // Recursively call to clear UI
        }

        return;
    }

    // Check if a model is selected
    if (!m_selectedModel.isEmpty()) {
        // Find the selected model in the active view
        bool found = false;
        for (const auto& model : activeView->m_models) {
            if (model.name == m_selectedModel) {
                found = true;

                m_updatingUI = true;

                // Update position
                updatePositionUI(model.position);

                // Update rotation
                updateRotationUI(model.rotation);

                // Update scale
                updateScaleUI(model.scale);

                m_updatingUI = false;

                break;
            }
        }

        // If the model wasn't found in this view, clear the selection
        if (!found) {
            m_selectedModel = "";
            updateUIFromSelection(); // Recursively call to clear UI
        }

        return;
    }

    // Check if a model node is selected
    if (!m_selectedModelName.isEmpty() && m_selectedNodeIndex >= 0) {
        // Find the selected model node in the active view
        bool found = false;
        for (const auto& model : activeView->m_models) {
            if (model.name == m_selectedModelName &&
                m_selectedNodeIndex < model.nodes.size()) {

                found = true;
                m_updatingUI = true;

                // Get the node
                const auto& node = model.nodes[m_selectedNodeIndex];

                // Update position
                updatePositionUI(node.position);

                // Update rotation
                updateRotationUI(node.rotation);

                // Update scale
                updateScaleUI(node.scale);

                m_updatingUI = false;
                break;
            }
        }

        // If the model node wasn't found in this view, clear the selection
        if (!found) {
            m_selectedModelName = "";
            m_selectedNodeIndex = -1;
            updateUIFromSelection(); // Recursively call to clear UI
        }
    }
}
void TransformManager::setTargetWidget(OpenGLWidget* w)
{
    m_targetWidget = w;   // assuming such a member exists
}
void TransformManager::onPointCloudDeselected(const QString& name)
{
    if (m_selectedPointCloud == name) {
        m_selectedPointCloud.clear();
        m_targetType = TargetType::None;  // or PointCloud if supporting multi-selection
        disableTransformControls();       // hides gizmo or disables UI
    }
}

void TransformManager::onModelDeselected(const QString& name)
{
    if (m_selectedModel == name) {
        m_selectedModel.clear();
        m_targetType = TargetType::None;  // or Model if supporting multi-selection
        disableTransformControls();       // hides gizmo or disables UI
    }
}
