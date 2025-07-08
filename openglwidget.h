
#ifndef OPENGLWIDGET_H
#define OPENGLWIDGET_H

#include <QtWidgets>
#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#include <QOpenGLBuffer>
#include <QOpenGLVertexArrayObject>
#include <QTimer>
#include <QVector3D>
#include <vector>
#include <QMouseEvent>
#include <QMatrix4x4>
#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#ifdef Q_OS_WIN
#include <Windows.h>
#endif
#include <GL/gl.h>
#include <GL/glu.h>
#include <QPainter>
#include <map>
#include "samplingprocessor.h"
#include "cluster.h"
#include <cilantro/clustering/kmeans.hpp>
#include <cilantro/utilities/point_cloud.hpp>
#include <cilantro/core/kd_tree.hpp>

class OpenGLWidget : public QOpenGLWidget, protected QOpenGLFunctions
{
    Q_OBJECT
public:
    friend class TransformManager;
    explicit OpenGLWidget(QWidget *parent = nullptr);
    ~OpenGLWidget();

    struct PointCloud {
        QString name;
        std::vector<QVector3D> points;
        std::vector<QVector3D> colors;
        QVector3D min, max;
        GLuint vbo;
        bool vboInitialized;
        bool visible;
        bool selected;
        int pointCount;
        std::vector<int> labels;

        // Add these members to support transforms
        QVector3D offset;    // Position offset
        QVector3D rotation;  // Rotation in degrees
        QVector3D scale;     // Scale factors

        // Constructor with defaults
        PointCloud() :
            vbo(0),
            vboInitialized(false),
            visible(true),
            selected(false),
            pointCount(0),
            offset(0.0f, 0.0f, 0.0f),
            rotation(0.0f, 0.0f, 0.0f),
            scale(1.0f, 1.0f, 1.0f) {}

        // Add a method to get the bounding box center
        QVector3D boundingBox() const {
            return (min + max) * 0.5f;
        }
        QString parentName;

        bool isMeshData = false;
        bool originalVisibility = true;

        // Add a method to update the bounding box after points change
        void updateBoundingBox() {
            if (points.empty()) {
                min = max = QVector3D(0, 0, 0);
                return;
            }

            min = max = points[0];
            for (const auto& point : points) {
                min.setX(qMin(min.x(), point.x()));
                min.setY(qMin(min.y(), point.y()));
                min.setZ(qMin(min.z(), point.z()));
                max.setX(qMax(max.x(), point.x()));
                max.setY(qMax(max.y(), point.y()));
                max.setZ(qMax(max.z(), point.z()));
            }
        }
    };

    // First, define Mesh
    struct Mesh {
        QString name;           // Add this field
        QString materialName;   // Add this field
        std::vector<QVector3D> vertices;
        std::vector<QVector3D> normals;
        std::vector<unsigned int> indices;
        GLuint vbo;
        GLuint ibo;
        bool vboInitialized;
        int vertexCount;
        int indexCount;

        // Constructor with defaults
        Mesh() :
            vbo(0),
            ibo(0),
            vboInitialized(false),
            vertexCount(0),
            indexCount(0) {}
        bool originalVisibility = true;
        QString parentName;
    };
    struct ModelNode {
        QString name;
        std::vector<OpenGLWidget::Mesh> meshes;  // Use the fully qualified type
        std::vector<int> children;
        int parentIndex;
        QVector3D position;
        QVector3D rotation;
        QVector3D scale;
        bool visible;
        bool selected;

        ModelNode() :
            parentIndex(-1),
            position(0.0f, 0.0f, 0.0f),
            rotation(0.0f, 0.0f, 0.0f),
            scale(1.0f, 1.0f, 1.0f),
            visible(true),
            selected(false) {}
    };

    struct Grid2D {
        int rows = 0;
        int columns = 0;
        float cellSize = 1.0f;
        bool visible = true;
        bool selected = false;
        QString name;
        GLuint vbo = 0;
        GLuint ibo = 0;
        bool vboInitialized = false;
        int vertexCount = 0;
        int indexCount = 0;
        QVector3D min;
        QVector3D max;
        float height =0.0f;
    };
    struct Grid3D {
        int rows = 0;
        int columns = 0;
        int layers = 0;
        float cellSize = 1.0f;
        bool visible = true;
        bool selected = false;
        QString name;
        GLuint vbo = 0;
        GLuint ibo = 0;
        bool vboInitialized = false;
        int vertexCount = 0;
        int indexCount = 0;
        QVector3D min;
        QVector3D max;
    };


    // Then define Model which uses Mesh
    struct Model {
        QString name;
        std::vector<ModelNode> nodes;
        std::vector<Mesh> meshes;  // Now Mesh is already defined when used here
        QVector3D min, max;
        bool visible;
        bool selected;

        // Add these members to support transforms
        QVector3D position;  // Position
        QVector3D rotation;  // Rotation in degrees
        QVector3D scale;     // Scale factors
        QVector3D offset;    // Additional offset for centering

        // Constructor with defaults
        Model() :
            visible(true),
            selected(false),
            position(0.0f, 0.0f, 0.0f),
            rotation(0.0f, 0.0f, 0.0f),
            scale(1.0f, 1.0f, 1.0f),
            offset(0.0f, 0.0f, 0.0f) {}
        QString parentName;
        bool originalVisibility = true;
    };

    // Add this struct for the getPointCloudMap method
    struct PointCloudData {
        QVector3D offset;
        QVector3D rotation;
        QVector3D scale;
    };

    struct ViewportCapture {
        QString name;
        float cameraDistance = 10.0f;
        QVector3D rotation = QVector3D(0.0f, 0.0f, 0.0f);
        QVector3D panOffset = QVector3D(0.0f, 0.0f, 0.0f);
        float zoom = 1.0f;
        bool selected = false;
        QMatrix4x4 projectionMatrix;
        QVector3D position;
        QVector3D target;
        QVector3D upVector;
        float xRotation;
        float yRotation;
    };

    void setIs2DMode(bool is2D) { m_is2DMode = is2D; update(); }
    bool is2DMode() const { return m_is2DMode; }
    void setIsPreview(bool isPreview) { m_isPreview = isPreview; }
    bool isPreview() const { return m_isPreview; }
    void setRenderingEnabled(bool enabled) { m_isRenderingEnabled = enabled; }
    bool isRenderingEnabled() const { return m_renderingEnabled; }

    std::vector<PointCloud> m_pointClouds;
    std::vector<Model> m_models;
    std::vector<Grid2D> m_grids2D;
    std::vector<Grid3D> m_grids3D;
    void adjustPreviewCamera();



    void updatePointCloud(const QString& name, const std::vector<QVector3D>& points, const std::vector<QVector3D>& colors, bool isMeshData = false);

    bool getPointCloudByName(const QString &name, std::vector<QVector3D> &points, std::vector<QVector3D> &colors);
    bool applyTransformToSelection(const QVector3D& position, const QVector3D& rotation, const QVector3D& scale);
    // Get transform information for selected items
    QList<QVariant> getSelectedItemsTransform() const;

    void add2DGrid(int rows, int columns, float cellSize, const QString& name);
    void clear2DGrids();
    void set2DGridVisibility(const QString& name, bool visible);
    bool is2DGridVisible(const QString& name) const;
    void set2DGridSelected(const QString& name, bool selected);
    bool is2DGridSelected(const QString& name) const;
    // const std::vector<Grid2D>& get2DGrids() const { return m_grids2D; }
    const std::vector<Grid2D>& get2DGrids() const;
    const std::vector<Grid2D>& getGrids2D() const;


    void add3DGrid(int rows, int columns, int layers, float cellSize, const QString& name);
    void clear3DGrids();
    void set3DGridVisibility(const QString& name, bool visible);
    bool is3DGridVisible(const QString& name) const;
    void set3DGridSelected(const QString& name, bool selected);
    bool is3DGridSelected(const QString& name) const;
    const std::vector<Grid3D>& get3DGrids() const { return m_grids3D; }

    void addPointCloud(const std::vector<QVector3D>& points, const std::vector<QVector3D>& colors, const QString& name);
    void addModel(const QString& filePath, const QString& name);
    void clearPointClouds();
    void clearModels();
    void setPointCloudVisibility(const QString& name, bool visible);
    void setModelVisibility(const QString& name, bool visible);
    bool isPointCloudVisible(const QString& name) const;
    bool isModelVisible(const QString& name) const;
    void setPointCloudSelected(const QString& name, bool selected);
    void setModelSelected(const QString& name, bool selected);
    bool isPointCloudSelected(const QString& name) const;
    bool isModelSelected(const QString& name) const;
    const std::vector<PointCloud>& getPointClouds() const { return m_pointClouds; }
    const std::vector<Model>& getModels() const { return m_models; }

    void setGridVisible(bool visible) { m_showGrid = visible; update(); }
    void setGridSize(float size) { m_gridSize = size; m_gridNeedsUpdate = true; update(); }
    void setGridSpacing(float spacing) { m_gridSpacing = spacing; m_gridNeedsUpdate = true; update(); }
    bool isGridVisible() const { return m_showGrid; }

    void setFrameRate(int fps) {
        if (fps > 0) timer->start(1000 / fps);
        else timer->stop();
    }
    // Sets the grid visibility

    void processPointCloud(const QString& name, const QString& operation);
    void convertMeshToPointCloud(const QString& name);
        // Add missing method
    void setCoordinateSystemVisible(bool visible) { m_showCoordinateSystem = visible; update(); }
    bool isCoordinateSystemVisible() const { return m_showCoordinateSystem; }

    // Gets the current grid visibility state
    void setAxisVisible(bool visible);
    // Gets the current axis visibility state
    bool isAxisVisible() const;

    void add2DGridAtHeight(int rows, int columns, float cellSize, const QString& name, float height);
    void handleMultiGridParameters(int rows, int columns, float cellSize, int duplicateCount, const std::vector<float>& heights, const QString& baseName);


    void setGridCellVisibility(const QString& gridName, int row, int col, bool visible);
    void clearGridCellSelections(const QString& gridName); // Added this method declaration
    void createPointCloudSection(const Grid2D& grid, const QString& gridName, int row, int col, const QString& sectionName);
    void removePointsFromCell(float xMin, float xMax, float zMin, float zMax, const QString& excludeSectionName);
    void updatePointCloudVBO(PointCloud& pc);

    // void updateObjectVisibilityBetweenGrids(); // Existing method

    // // New methods to handle splitting objects
    void splitPointCloudByHeight(const QString& name, float lowerHeight, float upperHeight, bool keepAbove);
    void splitModelByHeight(const QString& name, float lowerHeight, float upperHeight, bool keepAbove);
    void updateObjectVisibilityBetweenGrids();

    void convert2DTo3DGrid(const QString& name, int layers, float cellSize);

    //void clusterPointCloud(const QString& name, float voxelSize, float sampleFraction,float planeDistanceThreshold, float clusterEps,size_t maxPlaneIterations, size_t clusterMinPts);
    void clusterPointCloud(const QString& name, float voxelSize, float sampleFraction,
                           float planeDistanceThreshold, float clusterEps,
                           size_t maxPlaneIterations, size_t clusterMinPts,
                           float minPlaneInlierRatio);
    void identifySimilarClusters(const QString& name);

    //viewport
    void captureViewport(const QString& name);
    void addViewportCapture(const ViewportCapture& capture);

    void applyViewportCapture(const QString& name);
    QList<ViewportCapture> getViewportCaptures() const;
    QMatrix4x4 getProjectionMatrix() const;
    void setProjectionMatrix(const QMatrix4x4& matrix);

    void getCameraParameters(QVector3D& position, QVector3D& target, QVector3D& up) const;
    void setCameraParameters(const QVector3D& position, const QVector3D& target, const QVector3D& up);

    QVector3D getPanOffset() const { return m_panOffset; }
    void setPanOffset(const QVector3D& offset) { m_panOffset = offset; m_viewChanged = true; }

    void getRotation(float& xRot, float& yRot) const { xRot = m_xRotation; yRot = m_yRotation; }
    void setRotation(float xRot, float yRot) { m_xRotation = xRot; m_yRotation = yRot; m_viewChanged = true; }

    float getZoom() const { return m_zoom; }
    void setZoom(float zoom) { m_zoom = zoom; m_viewChanged = true; }
    void applySingleColor(const QString& name, const QVector3D& color);
    void applyGradientColor(const QString& name, const QVector3D& startColor, const QVector3D& endColor);
    void applyTransformToModel(const QString& name, const QVector3D& position, const QVector3D& rotation, const QVector3D& scale);
    void drawModelNode(Model& model, int nodeIndex);
    void initializeMeshVBO(Mesh& mesh);

    void setModelNodeVisibility(const QString& modelName, int nodeIndex, bool visible);

    bool isModelNodeVisible(const QString& modelName, int nodeIndex) const;
    void setModelNodeSelected(const QString& modelName, int nodeIndex, bool selected);
    bool isModelNodeSelected(const QString& modelName, int nodeIndex) const;
    bool deleteObject(int objectId = -1);
    void setOrthographicView();
    bool m_isPerspective = true;
    void resetCamera();
    void setViewChanged(bool changed);
    void setBottomView();
    void invalidateBuffers();
    void addLightweightModel(const QString& name,
                             const std::vector<QVector3D>& vertices,
                             const std::vector<QVector3D>& normals,
                             const std::vector<unsigned int>& indices);
        // custom rotation
    QVector3D m_rotationCenter;
    bool m_rotationCenterSet = false;
    void resetRotationCenter();
    QVector3D m_hoveredPoint;
    bool m_hasHoveredPoint = false;
    QVector3D findClosestPointOnGeometry(const QPoint& screenPos, bool& pointFound);
    bool m_selectingRotationCenter = false;  // <- Whether user is in point-selection mode
    QVector3D findClosestPointOrProjected(const QPoint& screenPos, bool& usedProjected);
    QVector3D getCursorProjectedPoint(const QPoint& screenPos, bool& success);
    QVector3D findClosestFBXVertex(const QPoint& screenPos, bool& pointFound);
    QVector3D findFallbackFBXPickPoint(const QPoint& screenPos, bool& pointFound);
    void drawFBXDebugPoints();
    QPoint m_mouseScreenPos;  // Stores last mouse position in screen space for hover comparison
    void updatePointCloudColors(const QString& name, const std::vector<QVector3D>& newColors);
    void updatePointCloudColorsWithMap(const QString& name, const std::map<int, QVector3D>& clusterColorMap);
    void updatePointCloudData(const PointCloud& updatedPC);
    QVector3D applyTransformations(const QVector3D& point, 
                                 const QVector3D& offset, 
                                 const QVector3D& rotation, 
                                 const QVector3D& scale) const;
    void removePointCloud(const QString& name);
    void removeModel(const QString& name);
    void remove2DGrid(const QString& name);
    void remove3DGrid(const QString& name);


    //custom rotation

public slots:
    void setFrontView();
    void setTopView();
    void setSideView();
    void setIsometricView();
    void setCenteredView(); // Centers on selected object


signals:
    void pointCloudSelected(const QString& name);
    void pointCloudTransformChanged(const QString& name);
    void modelSelected(const QString& name);
    void modelTransformChanged(const QString& name);
    void clicked();
    // Add new signals for node selection
    void modelNodeSelected(const QString& modelName, int nodeIndex);
    void modelNodeTransformChanged(const QString& modelName, int nodeIndex);
    void viewportChanged();

protected:
    void initializeGL() override;
    void paintGL() override;
    void resizeGL(int width, int height) override;
    // Add a method to access the point cloud map
    const std::map<QString, PointCloudData> getPointCloudMap() const;

    void mousePressEvent(QMouseEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override;
    void mouseReleaseEvent(QMouseEvent *event) override;
    void wheelEvent(QWheelEvent *event) override;

    void showEvent(QShowEvent *event) override;
    void hideEvent(QHideEvent *event) override;
    void resizeEvent(QResizeEvent *event) override;
    //custom rotation
    void contextMenuEvent(QContextMenuEvent *event) override;

private:
    QTimer *timer;
    bool m_showAxis = true;
    bool m_showCoordinateSystem = true;
    // Camera control variables
    QPointF m_lastPos;
    float m_xRotation = 0.0f;
    float m_yRotation = 0.0f;
    bool m_rotating = false;
    bool m_panning = false;
    float m_zoom = 1.0f;
    QVector3D m_cameraPosition = QVector3D(0.0f, 0.0f, 5.0f);
    QVector3D m_panOffset;

    QVector3D m_previewCameraPos = QVector3D(0.0f, 0.0f, 5.0f);
    QVector3D m_previewLookAt = QVector3D(0.0f, 0.0f, 0.0f);
    QVector3D m_previewUp = QVector3D(0.0f, 1.0f, 0.0f);

    bool m_isPreview = false;
    bool m_is2DMode = false;
    bool m_renderingEnabled = true;
    int m_frameRate = 60;
    // Rendering control flag
    bool m_isRenderingEnabled = true;

    // Drawing methods
    void calculateBoundingBox(const std::vector<QVector3D>& points, QVector3D& min, QVector3D& max);
    void drawPointCloud(PointCloud& pc);
    void drawModel(Model& model);
    void initializePointCloudVBO(PointCloud& pc);
    void initializeModelVBO(Model& model);
    void drawDefaultTriangle();
    void drawCoordinateSystem();

    // Grid drawing
    bool m_showGrid = true;
    bool m_autoResizeGrid = false;
    float m_gridSize = 10000.0f;
    float m_gridSpacing = 1.0f;
    bool m_gridNeedsUpdate = false;
    GLuint m_gridVBO = 0;
    GLuint m_gridIBO = 0;
    int m_gridVertexCount = 0;
    int m_gridIndexCount = 0;
    void updateGrid();
    void drawGrid();


    void draw2DGrid(Grid2D& grid);
    void initialize2DGridVBO(Grid2D& grid);

    void draw3DGrid(Grid3D& grid);
    void initialize3DGridVBO(Grid3D& grid);

    // Scene parameters
    QVector3D m_sceneCenter;
    float m_sceneRadius = 10000.0f;
    bool m_sceneBoundsValid = false;
    void updateSceneBounds();

    bool processAssimpNode(const aiNode* node, const aiScene* scene, Model& model, int parentIndex = -1);
    Mesh processAssimpMesh(const aiMesh* mesh, const aiScene* scene);

    void sampleMeshSurface(const Model& model, std::vector<QVector3D>& points, std::vector<QVector3D>& colors, int sampleCount);


    // Performance optimization
    bool m_viewChanged = true;

    //viewport
    QList<ViewportCapture> m_viewportCaptures;
    QMatrix4x4 m_projectionMatrix;

    QVector3D m_cameraTarget = QVector3D(0.0f, 0.0f, 0.0f);
    QVector3D m_cameraUp = QVector3D(0.0f, 1.0f, 0.0f);

    // custom rotation
    void updateProjectionMatrix();
    QImage m_axisIndicator;
};
#endif // OPENGLWIDGET_H
