#ifndef TRANSFORMMANAGER_H
#define TRANSFORMMANAGER_H

#include <QObject>
#include <QVector3D>
#include <QString>
#include <map>
#include <QTreeWidgetItem>
#include <QItemSelection>
#include<QRegularExpression>


class OpenGLWidget;
class MainWindow;
class QDoubleSpinBox;
class Ui_MainWindow;

class TransformManager : public QObject
{
    Q_OBJECT
    enum class TargetType { None, PointCloud, Model, Grid2D, Grid3D } m_targetType = TargetType::None;
public:
    explicit TransformManager(OpenGLWidget* openGLWidget, MainWindow* mainWindow, QObject* parent = nullptr);
    void setupConnections();
    void setTargetWidget(OpenGLWidget* w);
    OpenGLWidget* m_targetWidget = nullptr;
    void onPointCloudDeselected(const QString& name);
    void onModelDeselected(const QString& name);


public slots:
    void onPointCloudSelected(const QString& filePath);
    void onModelSelected(const QString& name);
    void onPointCloudTransformChanged(const QString& filePath);
    void enableTransformControls();
    void disableTransformControls();
    void onModelNodeSelected(const QString& modelName, int nodeIndex);
    // Position setters
    void setPositionX(double x);
    void setPositionY(double y);
    void setPositionZ(double z);

    // Rotation setters
    void setRotationX(double x);
    void setRotationY(double y);
    void setRotationZ(double z);

    // Scale setters
    void setScaleX(double x);
    void setScaleY(double y);
    void setScaleZ(double z);

    void onLockScaleCheckboxToggled(bool checked);
    void resetTransform();

private:
    void updateUIFromSelection();
    void onTreeItemSelectionChanged(QTreeWidgetItem* item);
    void updatePositionUI(const QVector3D& position);
    void updateRotationUI(const QVector3D& rotation);
    void updateScaleUI(const QVector3D& scale);
    QDoubleSpinBox* findSpinBox(const QString& objectName);
    void applyScale(double newValue, int axis, OpenGLWidget* activeView = nullptr);
    QString m_selectedModelName;
    int m_selectedNodeIndex;


    OpenGLWidget* m_openGLWidget;
    MainWindow* m_mainWindow;
    QString m_selectedPointCloud;
    QString m_selectedModel;
    bool m_lockScaleProportions;
    bool m_updatingUI;
};

#endif // TRANSFORMMANAGER_H
