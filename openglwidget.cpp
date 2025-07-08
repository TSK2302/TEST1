#include "openglwidget.h"
#include "samplingprocessor.h"
#include "colorassigner.h"
#include <vector>
#include <QDebug>
#include <QPainter>
#include <QtMath>
#include <random>
#include <QApplication>
#include <QDebug>
#include <QMetaObject>
#include <thread>
#include <chrono>
#include <atomic>
// struct Vertex {
//     QVector3D position;
// };

OpenGLWidget::OpenGLWidget(QWidget *parent)
    : QOpenGLWidget(parent)
{
    timer = new QTimer(this);
    connect(timer, &QTimer::timeout, this, QOverload<>::of(&QOpenGLWidget::update));
    timer->start(33);
    setContextMenuPolicy(Qt::DefaultContextMenu);
    setMouseTracking(true);
    setFocusPolicy(Qt::StrongFocus);
}
OpenGLWidget::~OpenGLWidget()
{
    timer->stop();
    makeCurrent();

    for (auto& pc : m_pointClouds) {
        if (pc.vboInitialized) {
            GLuint buffer = pc.vbo;
            glDeleteBuffers(1, &buffer);
            pc.vbo = buffer;
        }
    }

    for (auto& model : m_models) {
        for (auto& mesh : model.meshes) {
            if (mesh.vboInitialized) {
                glDeleteBuffers(1, &mesh.vbo);
                glDeleteBuffers(1, &mesh.ibo);
            }
        }
    }
    for (auto& grid : m_grids2D) {
        if (grid.vboInitialized) {
            glDeleteBuffers(1, &grid.vbo);
            glDeleteBuffers(1, &grid.ibo);
        }
    }

    for (auto& grid : m_grids3D) {
        if (grid.vboInitialized) {
            glDeleteBuffers(1, &grid.vbo);
            glDeleteBuffers(1, &grid.ibo);
        }
    }

    if (m_gridVBO) {
        glDeleteBuffers(1, &m_gridVBO);
    }
    if (m_gridIBO) {
        glDeleteBuffers(1, &m_gridIBO);
    }

    doneCurrent();
}


void OpenGLWidget::initializeGL()
{
    initializeOpenGLFunctions();

    // Set a dark background for contrast
    glClearColor(0.078f, 0.078f, 0.078f, 1.0f);

    // Enable depth testing
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_POINT_SMOOTH);
    glDepthFunc(GL_LEQUAL);

    // Enable point smoothing for better point cloud rendering

    glHint(GL_POINT_SMOOTH_HINT, GL_NICEST);

    // Enable lighting and material properties
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_LIGHT1); // Add a second light for better coverage
    glEnable(GL_COLOR_MATERIAL);

    // Configure first light (from above-right)
    GLfloat light0_pos[] = { 1.0f, 1.0f, 1.0f, 0.0f }; // Directional light
    GLfloat light0_ambient[] = { 0.4f, 0.4f, 0.4f, 1.0f }; // Brighter ambient
    GLfloat light0_diffuse[] = { 0.8f, 0.8f, 0.8f, 1.0f }; // Brighter diffuse
    glLightfv(GL_LIGHT0, GL_POSITION, light0_pos);
    glLightfv(GL_LIGHT0, GL_AMBIENT, light0_ambient);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, light0_diffuse);

    // Configure second light (from below-left)
    GLfloat light1_pos[] = { -1.0f, -1.0f, -1.0f, 0.0f }; // Opposite direction
    GLfloat light1_ambient[] = { 0.2f, 0.2f, 0.2f, 1.0f };
    GLfloat light1_diffuse[] = { 0.5f, 0.5f, 0.5f, 1.0f };
    glLightfv(GL_LIGHT1, GL_POSITION, light1_pos);
    glLightfv(GL_LIGHT1, GL_AMBIENT, light1_ambient);
    glLightfv(GL_LIGHT1, GL_DIFFUSE, light1_diffuse);

    // Set global ambient light for minimum visibility
    GLfloat global_ambient[] = { 0.3f, 0.3f, 0.3f, 1.0f };
    glLightModelfv(GL_LIGHT_MODEL_AMBIENT, global_ambient);

    // Initialize grid buffers
    glGenBuffers(1, &m_gridVBO);
    glGenBuffers(1, &m_gridIBO);
    m_gridNeedsUpdate = true;
    if (m_isPreview) {
        setMouseTracking(false);
        setFocusPolicy(Qt::NoFocus);
        m_showGrid = false;
        m_showCoordinateSystem = false;
        timer->stop();
    } else {
        timer->start(1000 / 60);
    }

    qDebug() << "OpenGL Widget initialized";
    qDebug() << "OpenGL Version:" << reinterpret_cast<const char*>(glGetString(GL_VERSION));
}


void OpenGLWidget::resizeGL(int width, int height)
{
    glViewport(0, 0, width, height);
    m_viewChanged = true;
}

void OpenGLWidget::mouseReleaseEvent(QMouseEvent *event)
{
    if (m_isPreview) return;
    if (event->button() == Qt::LeftButton) {
        m_rotating = false;
    } else if (event->button() == Qt::RightButton) {
        m_panning = false;
    }
}


// Modified wheelEvent function with smoother zoom
void OpenGLWidget::wheelEvent(QWheelEvent *event)
{
    if (m_isPreview) return;
    QPoint numDegrees = event->angleDelta() / 8;
    if (!numDegrees.isNull()) {
        // Get current camera distance to scene center
        float sceneSize = m_sceneBoundsValid ? m_sceneRadius : 10.0f;
        float currentDistance;

        if (m_sceneBoundsValid) {
            // Calculate actual distance from camera to scene center
            currentDistance = m_sceneRadius * 2.0f / m_zoom;
        } else {
            currentDistance = 5.0f / m_zoom;
        }

        // Calculate zoom increment based on current distance
        float zoomFactor;

        // SMOOTHER ZOOM: Reduce the zoom factor for gentler movement
        if (numDegrees.y() > 0) {
            // Zoom in - move closer (reduced from 0.8f to 0.95f)
            zoomFactor = 0.95f;  // Move only 5% closer per wheel click
        } else {
            // Zoom out - move away (reduced from 1.25f to 1.05f)
            zoomFactor = 1.05f; // Move only 5% away per wheel click
        }

        // For extremely close zooms, make movement even gentler
        if (m_zoom > 100.0f && numDegrees.y() > 0) {
            zoomFactor = 0.98f;  // Even gentler when zoomed in far
        }

        // For extremely far zooms, make movement more responsive
        if (m_zoom < 0.1f && numDegrees.y() < 0) {
            zoomFactor = 1.1f;  // More responsive when zoomed out far
        }

        // Apply zoom factor to current distance
        currentDistance *= zoomFactor;

        // Limit range to prevent numerical issues
        float minDistance = 0.0001f * sceneSize;
        float maxDistance = 100.0f * sceneSize;

        currentDistance = qMax(minDistance, qMin(currentDistance, maxDistance));

        // Convert back to zoom factor
        if (m_sceneBoundsValid) {
            m_zoom = m_sceneRadius * 2.0f / currentDistance;
        } else {
            m_zoom = 5.0f / currentDistance;
        }

        m_viewChanged = true;
        update();
    }
}


void OpenGLWidget::addPointCloud(const std::vector<QVector3D>& points, const std::vector<QVector3D>& colors, const QString& name)
{
    PointCloud pc;
    pc.points = points;
    pc.colors = colors;
    pc.name = name;
    pc.visible = true;
    pc.selected = false;
    pc.pointCount = points.size();

    calculateBoundingBox(points, pc.min, pc.max);
    qDebug() << "Point cloud '" << name << "' bounds: min =" << pc.min << ", max =" << pc.max;


    // Improve centering logic
    QVector3D center = (pc.min + pc.max) * 0.5f;

    // Calculate max extent using separate comparisons
    float maxExtent = qMax(
        qMax(pc.max.x() - pc.min.x(), pc.max.y() - pc.min.y()),
        pc.max.z() - pc.min.z()
        );

    // Scale to fit view if points are too small or too spread out
    float scaleFactor = 1.0f;
    if (maxExtent > 0) {
        scaleFactor = 10.0f / maxExtent;
    }

    for (auto& p : pc.points) {
        p -= center;  // Center the points
        p *= scaleFactor;  // Scale to a reasonable size
    }

    calculateBoundingBox(pc.points, pc.min, pc.max);
    qDebug() << "Adjusted point cloud bounds: min =" << pc.min << ", max =" << pc.max;
    if (m_isPreview) {
        adjustPreviewCamera();
    }

    m_pointClouds.push_back(pc);
    m_sceneBoundsValid = false;
    qDebug() << "Point cloud '" << name << "' added with" << points.size() << "points";
    update();


}
//preview
void OpenGLWidget::adjustPreviewCamera()
{
    if (!m_isPreview) return;

    // If there are no point clouds or models, use default camera settings
    if (m_pointClouds.empty() && m_models.empty() && m_grids2D.empty() && m_grids3D.empty()) {
        m_previewCameraPos = QVector3D(0.0f, 0.0f, 5.0f);
        m_previewLookAt = QVector3D(0.0f, 0.0f, 0.0f);
        m_previewUp = QVector3D(0.0f, 1.0f, 0.0f);
        return;
    }

    // Update scene bounds if necessary
    if (!m_sceneBoundsValid) {
        updateSceneBounds();
    }

    // Set up camera parameters for preview - position the camera at an isometric angle
    float distance = m_sceneRadius * 2.0f;
    m_previewCameraPos = m_sceneCenter + QVector3D(distance, distance * 0.7f, distance);
    m_previewLookAt = m_sceneCenter;
    m_previewUp = QVector3D(0.0f, 1.0f, 0.0f);
}


void OpenGLWidget::addModel(const QString& filePath, const QString& name)
{
    Assimp::Importer importer;
    const aiScene* scene = importer.ReadFile(filePath.toStdString(),
                                             aiProcess_Triangulate | aiProcess_GenNormals | aiProcess_JoinIdenticalVertices);

    if (!scene || scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE || !scene->mRootNode) {
        qDebug() << "Error loading model:" << importer.GetErrorString();
        return;
    }

    Model model;
    model.name = name;
    model.visible = true;
    model.selected = false;
    model.position = QVector3D(0.0f, 0.0f, 0.0f);
    model.rotation = QVector3D(0.0f, 0.0f, 0.0f);
    model.scale = QVector3D(1.0f, 1.0f, 1.0f);
    model.offset = QVector3D(0.0f, 0.0f, 0.0f);
    processAssimpNode(scene->mRootNode, scene, model);
    // Calculate overall bounding box
    std::vector<QVector3D> allVertices;
    for (const auto& node : model.nodes) {
        for (const auto& mesh : node.meshes) {
            allVertices.insert(allVertices.end(), mesh.vertices.begin(), mesh.vertices.end());
        }
    }

    if (!allVertices.empty()) {
        calculateBoundingBox(allVertices, model.min, model.max);

        // Center the model if needed
        QVector3D center = (model.min + model.max) * 0.5f;
        if (center.length() > 1.0f) {
            model.offset = -center;  // Store the offset for rendering
        }

        m_models.push_back(model);
        m_sceneBoundsValid = false;
        if (m_isPreview) {
            for (auto& node : model.nodes) {
                for (auto& mesh : node.meshes) {
                    initializeMeshVBO(mesh);  // <-- ensure this function exists and initializes mesh.vbo and mesh.ibo
                }
            }
        }
        qDebug() << "Model '" << name << "' added with" << model.nodes.size()
                 << "nodes and" << allVertices.size() << "vertices";

        // Check file extension to see if it's an FBX file
        QFileInfo fileInfo(filePath);
        QString extension = fileInfo.suffix().toLower();
        if (extension == "fbx") {
            qDebug() << "FBX model loaded with hierarchical structure";
        }

        update();
    } else {
        qDebug() << "Model contains no vertices!";
    }
}

// Implement the getPointCloudMap method in openglwidget.cpp

const std::map<QString, OpenGLWidget::PointCloudData> OpenGLWidget::getPointCloudMap() const
{
    std::map<QString, PointCloudData> result;

    for (const auto& pc : m_pointClouds) {
        PointCloudData data;
        data.offset = pc.offset;
        data.rotation = pc.rotation;
        data.scale = pc.scale;
        result[pc.name] = data;
    }

    return result;
}



void OpenGLWidget::clearPointClouds()
{
    makeCurrent();
    for (auto& pc : m_pointClouds) {
        if (pc.vboInitialized) {
            GLuint buffer = pc.vbo;
            glDeleteBuffers(1, &buffer);
            pc.vbo = buffer;
        }
    }
    m_pointClouds.clear();
    m_sceneBoundsValid = false;
    update();
    doneCurrent();
}


void OpenGLWidget::clearModels()
{
    makeCurrent();
    for (auto& model : m_models) {
        for (auto& mesh : model.meshes) {
            if (mesh.vboInitialized) {
                glDeleteBuffers(1, &mesh.vbo);
                glDeleteBuffers(1, &mesh.ibo);
            }
        }
    }
    m_models.clear();
    m_sceneBoundsValid = false;
    update();
    doneCurrent();
}


// Simplified drawPointCloud without any zoom-based modifications
void OpenGLWidget::drawPointCloud(PointCloud& pc)
{
    if (!pc.vboInitialized) {
        initializePointCloudVBO(pc);
    }

    if (pc.pointCount == 0 || !pc.vboInitialized) return;
    // Apply transformations

    GLboolean lightingEnabled = glIsEnabled(GL_LIGHTING);
    if (lightingEnabled) glDisable(GL_LIGHTING);

    // Apply transformations
    glPushMatrix();

    // Calculate center of the point cloud for rotation
    QVector3D center = (pc.min + pc.max) * 0.5f;

    // 1. Translate to center
    glTranslatef(center.x(), center.y(), center.z());

    // 2. Apply offset (position)
    glTranslatef(pc.offset.x(), pc.offset.y(), pc.offset.z());

    // 3. Apply rotation
    glRotatef(pc.rotation.x(), 1.0f, 0.0f, 0.0f);
    glRotatef(pc.rotation.y(), 0.0f, 1.0f, 0.0f);
    glRotatef(pc.rotation.z(), 0.0f, 0.0f, 1.0f);

    // 4. Apply scaling
    glScalef(pc.scale.x(), pc.scale.y(), pc.scale.z());

    // 5. Translate back from center
    glTranslatef(-center.x(), -center.y(), -center.z());

    // Draw points with fixed size
    glEnableClientState(GL_VERTEX_ARRAY);
    glEnableClientState(GL_COLOR_ARRAY);

    glBindBuffer(GL_ARRAY_BUFFER, pc.vbo);

    const int stride = 6 * sizeof(float);
    glVertexPointer(3, GL_FLOAT, stride, 0);
    glColorPointer(3, GL_FLOAT, stride, (void*)(3 * sizeof(float)));

    glDrawArrays(GL_POINTS, 0, pc.pointCount);

    glDisableClientState(GL_COLOR_ARRAY);
    glDisableClientState(GL_VERTEX_ARRAY);
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    // Draw bounding box if selected
    if (pc.selected) {
        glLineWidth(1.0f);
        glColor3f(1.0f, 1.0f, 0.0f); // Yellow
        glBegin(GL_LINES);
        auto addLine = [&](float x1, float y1, float z1, float x2, float y2, float z2) {
            glVertex3f(x1, y1, z1);
            glVertex3f(x2, y2, z2);
        };
        float x1 = pc.min.x(), x2 = pc.max.x();
        float y1 = pc.min.y(), y2 = pc.max.y();
        float z1 = pc.min.z(), z2 = pc.max.z();
        addLine(x1, y1, z1, x2, y1, z1);
        addLine(x2, y1, z1, x2, y2, z1);
        addLine(x2, y2, z1, x1, y2, z1);
        addLine(x1, y2, z1, x1, y1, z1);
        addLine(x1, y1, z2, x2, y1, z2);
        addLine(x2, y1, z2, x2, y2, z2);
        addLine(x2, y2, z2, x1, y2, z2);
        addLine(x1, y2, z2, x1, y1, z2);
        addLine(x1, y1, z1, x1, y1, z2);
        addLine(x2, y1, z1, x2, y1, z2);
        addLine(x2, y2, z1, x2, y2, z2);
        addLine(x1, y2, z1, x1, y2, z2);
        glEnd();
    }

    // Restore the transformation matrix
    glPopMatrix();

    if (lightingEnabled) glEnable(GL_LIGHTING);
}

void OpenGLWidget::setPointCloudVisibility(const QString& name, bool visible)
{
    for (auto& pc : m_pointClouds) {
        if (pc.name == name) {
            pc.visible = visible;
            m_sceneBoundsValid = false;
            update();
            return;
        }
    }
    qDebug() << "Point cloud with name '" << name << "' not found.";
}


void OpenGLWidget::setModelVisibility(const QString& name, bool visible)
{
    for (auto& model : m_models) {
        if (model.name == name) {
            model.visible = visible;
            m_sceneBoundsValid = false;
            update();
            return;
        }
    }
    qDebug() << "Model with name '" << name << "' not found.";
}

bool OpenGLWidget::isPointCloudVisible(const QString& name) const
{
    for (const auto& pc : m_pointClouds) {
        if (pc.name == name) {
            return pc.visible;
        }
    }
    return false;
}


bool OpenGLWidget::isModelVisible(const QString& name) const
{
    for (const auto& model : m_models) {
        if (model.name == name) {
            return model.visible;
        }
    }
    return false;
}

void OpenGLWidget::setPointCloudSelected(const QString& name, bool selected)
{
    for (auto& pc : m_pointClouds) {
        if (pc.name == name) {
            pc.selected = selected;
            if (selected) {
                emit pointCloudSelected(name);
            }
            update();
            return;
        }
    }
    qDebug() << "Point cloud with name '" << name << "' not found.";
}

void OpenGLWidget::setModelSelected(const QString& name, bool selected)
{
    for (auto& model : m_models) {
        if (model.name == name) {
            model.selected = selected;
            if (selected) {
                emit modelSelected(name);
            }

            update();
            return;
        }
    }
    qDebug() << "Model with name '" << name << "' not found.";
}


bool OpenGLWidget::isPointCloudSelected(const QString& name) const
{
    for (const auto& pc : m_pointClouds) {
        if (pc.name == name) {
            return pc.selected;
        }
    }
    return false;
}
// Implementation of model node visibility and selection methods
void OpenGLWidget::setModelNodeVisibility(const QString& modelName, int nodeIndex, bool visible)
{
    for (auto& model : m_models) {
        if (model.name == modelName) {
            if (nodeIndex >= 0 && nodeIndex < static_cast<int>(model.nodes.size())) {
                model.nodes[nodeIndex].visible = visible;
                update();
            }
            break;
        }
    }
}

bool OpenGLWidget::isModelNodeVisible(const QString& modelName, int nodeIndex) const
{
    for (const auto& model : m_models) {
        if (model.name == modelName) {
            if (nodeIndex >= 0 && nodeIndex < static_cast<int>(model.nodes.size())) {
                return model.nodes[nodeIndex].visible;
            }
            break;
        }
    }
    return false;
}

void OpenGLWidget::setModelNodeSelected(const QString& modelName, int nodeIndex, bool selected)
{
    for (auto& model : m_models) {
        if (model.name == modelName) {
            if (nodeIndex >= 0 && nodeIndex < static_cast<int>(model.nodes.size())) {
                model.nodes[nodeIndex].selected = selected;

                // Emit signal when a node is selected
                if (selected) {
                    emit modelNodeSelected(modelName, nodeIndex);
                }

                update();
            }
            break;
        }
    }
}

bool OpenGLWidget::isModelNodeSelected(const QString& modelName, int nodeIndex) const
{
    for (const auto& model : m_models) {
        if (model.name == modelName) {
            if (nodeIndex >= 0 && nodeIndex < static_cast<int>(model.nodes.size())) {
                return model.nodes[nodeIndex].selected;
            }
            break;
        }
    }
    return false;
}

bool OpenGLWidget::isModelSelected(const QString& name) const
{
    for (const auto& model : m_models) {
        if (model.name == name) {
            return model.selected;
        }
    }
    return false;
}


void OpenGLWidget::calculateBoundingBox(const std::vector<QVector3D>& points, QVector3D& min, QVector3D& max)
{
    if (points.empty()) {
        min = QVector3D(0, 0, 0);
        max = QVector3D(0, 0, 0);
        return;
    }
    min = points[0];
    max = points[0];
    for (const auto& point : points) {
        min.setX(qMin(min.x(), point.x()));
        min.setY(qMin(min.y(), point.y()));
        min.setZ(qMin(min.z(), point.z()));
        max.setX(qMax(max.x(), point.x()));
        max.setY(qMax(max.y(), point.y()));
        max.setZ(qMax(max.z(), point.z()));
    }
}

void OpenGLWidget::updateSceneBounds()
{
    if (m_sceneBoundsValid) return;

    bool firstItem = true;
    QVector3D combinedMin, combinedMax;

    for (const auto& pc : m_pointClouds) {
        if (pc.visible && pc.pointCount > 0) {
            if (firstItem) {
                combinedMin = pc.min;
                combinedMax = pc.max;
                firstItem = false;
            } else {
                combinedMin.setX(qMin(combinedMin.x(), pc.min.x()));
                combinedMin.setY(qMin(combinedMin.y(), pc.min.y()));
                combinedMin.setZ(qMin(combinedMin.z(), pc.min.z()));
                combinedMax.setX(qMax(combinedMax.x(), pc.max.x()));
                combinedMax.setY(qMax(combinedMax.y(), pc.max.y()));
                combinedMax.setZ(qMax(combinedMax.z(), pc.max.z()));
            }
        }
    }

    for (const auto& model : m_models) {
        if (model.visible && !model.meshes.empty()) {
            if (firstItem) {
                combinedMin = model.min;
                combinedMax = model.max;
                firstItem = false;
            } else {
                combinedMin.setX(qMin(combinedMin.x(), model.min.x()));
                combinedMin.setY(qMin(combinedMin.y(), model.min.y()));
                combinedMin.setZ(qMin(combinedMin.z(), model.min.z()));
                combinedMax.setX(qMax(combinedMax.x(), model.max.x()));
                combinedMax.setY(qMax(combinedMax.y(), model.max.y()));
                combinedMax.setZ(qMax(combinedMax.z(), model.max.z()));
            }
        }
    }

    for (const auto& grid : m_grids2D) {
        if (grid.visible && grid.rows > 0 && grid.columns > 0) {
            if (firstItem) {
                combinedMin = grid.min;
                combinedMax = grid.max;
                firstItem = false;
            } else {
                combinedMin.setX(qMin(combinedMin.x(), grid.min.x()));
                combinedMin.setY(qMin(combinedMin.y(), grid.min.y()));
                combinedMin.setZ(qMin(combinedMin.z(), grid.min.z()));
                combinedMax.setX(qMax(combinedMax.x(), grid.max.x()));
                combinedMax.setY(qMax(combinedMax.y(), grid.max.y()));
                combinedMax.setZ(qMax(combinedMax.z(), grid.max.z()));
            }
        }
    }

    for (const auto& grid : m_grids3D) {
        if (grid.visible && grid.rows > 0 && grid.columns > 0 && grid.layers > 0) {
            if (firstItem) {
                combinedMin = grid.min;
                combinedMax = grid.max;
                firstItem = false;
            } else {
                combinedMin.setX(qMin(combinedMin.x(), grid.min.x()));
                combinedMin.setY(qMin(combinedMin.y(), grid.min.y()));
                combinedMin.setZ(qMin(combinedMin.z(), grid.min.z()));
                combinedMax.setX(qMax(combinedMax.x(), grid.max.x()));
                combinedMax.setY(qMax(combinedMax.y(), grid.max.y()));
                combinedMax.setZ(qMax(combinedMax.z(), grid.max.z()));
            }
        }
    }




    if (m_isPreview) {
        adjustPreviewCamera();
    }

    if (!firstItem) {
        m_sceneCenter = (combinedMin + combinedMax) * 0.5f;
        float dx = combinedMax.x() - combinedMin.x();
        float dy = combinedMax.y() - combinedMin.y();
        float dz = combinedMax.z() - combinedMin.z();

        m_sceneRadius = qSqrt(dx*dx + dy*dy + dz*dz) * 0.5f;
        m_sceneRadius = qMax(m_sceneRadius, 0.1f);
    } else {
        m_sceneCenter = QVector3D(0, 0, 0);
        m_sceneRadius = 10.0f;
    }

    m_sceneBoundsValid = true;
    m_gridNeedsUpdate = true;
}



void OpenGLWidget::initializePointCloudVBO(PointCloud& pc)
{
    if (pc.vboInitialized || pc.points.empty()) return;

    glGenBuffers(1, &pc.vbo);
    glBindBuffer(GL_ARRAY_BUFFER, pc.vbo);

    std::vector<float> vertexData;
    vertexData.reserve(pc.points.size() * 6);

    for (size_t i = 0; i < pc.points.size(); ++i) {
        vertexData.push_back(pc.points[i].x());
        vertexData.push_back(pc.points[i].y());
        vertexData.push_back(pc.points[i].z());

        if (i < pc.colors.size()) {
            vertexData.push_back(pc.colors[i].x());
            vertexData.push_back(pc.colors[i].y());
            vertexData.push_back(pc.colors[i].z());
        } else {
            vertexData.push_back(1.0f);
            vertexData.push_back(1.0f);
            vertexData.push_back(1.0f);
        }
    }

    glBufferData(GL_ARRAY_BUFFER, vertexData.size() * sizeof(float), vertexData.data(), GL_STATIC_DRAW);
    pc.vboInitialized = true;
    pc.pointCount = pc.points.size();
}

// Add these methods to openglwidget.cpp:
void OpenGLWidget::setAxisVisible(bool visible)
{
    // Only update if the state has changed
    if (m_showAxis != visible) {
        m_showAxis = visible;
        update();
    }
}

bool OpenGLWidget::isAxisVisible() const
{
    return m_showAxis;
}

void OpenGLWidget::initializeModelVBO(Model& model)
{
    for (auto& mesh : model.meshes) {
        if (mesh.vboInitialized || mesh.vertices.empty()) continue;

        glGenBuffers(1, &mesh.vbo);
        glGenBuffers(1, &mesh.ibo);

        glBindBuffer(GL_ARRAY_BUFFER, mesh.vbo);
        std::vector<float> vertexData;
        vertexData.reserve(mesh.vertices.size() * 6);

        for (size_t i = 0; i < mesh.vertices.size(); ++i) {
            vertexData.push_back(mesh.vertices[i].x());
            vertexData.push_back(mesh.vertices[i].y());
            vertexData.push_back(mesh.vertices[i].z());

            if (i < mesh.normals.size()) {
                vertexData.push_back(mesh.normals[i].x());
                vertexData.push_back(mesh.normals[i].y());
                vertexData.push_back(mesh.normals[i].z());
            } else {
                vertexData.push_back(0.0f);
                vertexData.push_back(0.0f);
                vertexData.push_back(1.0f);
            }
        }

        glBufferData(GL_ARRAY_BUFFER, vertexData.size() * sizeof(float), vertexData.data(), GL_STATIC_DRAW);

        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, mesh.ibo);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, mesh.indices.size() * sizeof(unsigned int), mesh.indices.data(), GL_STATIC_DRAW);

        mesh.vboInitialized = true;
        mesh.vertexCount = mesh.vertices.size();
        mesh.indexCount = mesh.indices.size();

        glBindBuffer(GL_ARRAY_BUFFER, 0);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
    }
}
// Completely revised paintGL function with proper visualization and infinite zoom

// --- Enhanced paintGL with Efficient FBX Projection for Hovering ---
void OpenGLWidget::paintGL()
{
    if (!m_renderingEnabled) {
        glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        return;
    }

    makeCurrent();

    if (!m_sceneBoundsValid) updateSceneBounds();
    if (m_gridNeedsUpdate) updateGrid();

    glClearColor(0.078f, 0.078f, 0.078f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    float aspectRatio = static_cast<float>(width()) / height();
    gluPerspective(45.0f, aspectRatio, 0.1f, 1000.0f);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    GLfloat light_pos[] = { 1.0f, 1.0f, 1.0f, 0.0f };
    glLightfv(GL_LIGHT0, GL_POSITION, light_pos);

    QVector3D center = m_rotationCenterSet ? m_rotationCenter : m_sceneCenter;

    if (!m_isPreview && m_sceneBoundsValid) {
        float cameraDistance = m_sceneRadius * 2.0f / m_zoom;
        QVector3D eye = center + QVector3D(0.0f, 0.0f, cameraDistance);
        QVector3D panAdjustedCenter = center + m_panOffset;

        gluLookAt(eye.x() + m_panOffset.x(), eye.y() + m_panOffset.y(), eye.z(),
                  panAdjustedCenter.x(), panAdjustedCenter.y(), panAdjustedCenter.z(),
                  0.0f, 1.0f, 0.0f);

        glTranslatef(center.x() + m_panOffset.x(), center.y() + m_panOffset.y(), center.z());

        if (m_xRotation > 360.0f) m_xRotation -= 360.0f;
        if (m_xRotation < -360.0f) m_xRotation += 360.0f;
        if (m_yRotation > 360.0f) m_yRotation -= 360.0f;
        if (m_yRotation < -360.0f) m_yRotation += 360.0f;

        glRotatef(m_xRotation, 1.0f, 0.0f, 0.0f);
        glRotatef(m_yRotation, 0.0f, 1.0f, 0.0f);
        glTranslatef(-(center.x() + m_panOffset.x()), -(center.y() + m_panOffset.y()), -center.z());
    } else {
        glTranslatef(m_panOffset.x(), m_panOffset.y(), -5.0f * m_zoom);
        glRotatef(m_xRotation, 1.0f, 0.0f, 0.0f);
        glRotatef(m_yRotation, 0.0f, 1.0f, 0.0f);
    }

    if (m_showGrid) drawGrid();

    glPointSize(2.0f);
    for (auto& pc : m_pointClouds) {
        if (!pc.points.empty() && pc.visible) drawPointCloud(pc);
    }
    for (auto& model : m_models) if (model.visible) drawModel(model);
    for (auto& grid : m_grids2D) if (grid.visible) draw2DGrid(grid);
    for (auto& grid : m_grids3D) if (grid.visible) draw3DGrid(grid);

    if (m_selectingRotationCenter && m_hasHoveredPoint) {
        glDisable(GL_LIGHTING);
        glEnable(GL_POINT_SMOOTH);
        glPointSize(20.0f);
        glDepthFunc(GL_ALWAYS);
        glColor3f(1.0f, 1.0f, 0.0f);
        glBegin(GL_POINTS);
        glVertex3f(m_hoveredPoint.x(), m_hoveredPoint.y(), m_hoveredPoint.z());
        glEnd();
        glDepthFunc(GL_LESS);
        glDisable(GL_POINT_SMOOTH);
        glEnable(GL_LIGHTING);
    }

    if (m_rotationCenterSet) {
        glDisable(GL_LIGHTING);
        glEnable(GL_POINT_SMOOTH);
        glPointSize(22.0f);
        glDepthFunc(GL_ALWAYS);
        glColor3f(1.0f, 0.0f, 1.0f);
        glBegin(GL_POINTS);
        glVertex3f(m_rotationCenter.x(), m_rotationCenter.y(), m_rotationCenter.z());
        glEnd();
        glDepthFunc(GL_LESS);
        glDisable(GL_POINT_SMOOTH);
        glEnable(GL_LIGHTING);
    }

    if (m_showAxis && width() > 50 && height() > 50) drawCoordinateSystem();
    m_viewChanged = false;
}



void OpenGLWidget::updateGrid()
{
    if (!m_gridNeedsUpdate) return;

    float gridSize = m_gridSize;
    float gridSpacing = m_gridSpacing;

    if (m_autoResizeGrid && m_sceneBoundsValid) {
        gridSize = m_sceneRadius * 4.0f;

        float roughSpacing = gridSize / 20.0f;
        float magnitude = pow(10, floor(log10(roughSpacing)));
        float normalized = roughSpacing / magnitude;

        if (normalized < 1.5f)
            gridSpacing = magnitude;
        else if (normalized < 3.5f)
            gridSpacing = 2.0f * magnitude;
        else if (normalized < 7.5f)
            gridSpacing = 5.0f * magnitude;
        else
            gridSpacing = 10.0f * magnitude;
    }

    int lineCount = static_cast<int>(gridSize / gridSpacing) + 1;
    // lineCount = qMin(lineCount, 100);

    float start = -lineCount * gridSpacing;
    float end = lineCount * gridSpacing;

    std::vector<float> vertices;
    std::vector<unsigned int> indices;
    std::vector<float> colors;

    int linesPerAxis = 2 * lineCount + 1;
    int totalVertices = 4 * linesPerAxis;

    vertices.reserve(totalVertices * 3);
    colors.reserve(totalVertices * 4);
    indices.reserve(totalVertices);

    unsigned int index = 0;

    auto addGridLine = [&](float x1, float y1, float z1, float x2, float y2, float z2, float r, float g, float b, float a) {
        vertices.push_back(x1);
        vertices.push_back(y1);
        vertices.push_back(z1);

        vertices.push_back(x2);
        vertices.push_back(y2);
        vertices.push_back(z2);

        colors.push_back(r);
        colors.push_back(g);
        colors.push_back(b);
        colors.push_back(a);

        colors.push_back(r);
        colors.push_back(g);
        colors.push_back(b);
        colors.push_back(a);

        indices.push_back(index++);
        indices.push_back(index++);
    };

    addGridLine(start, 0.0f, 0.0f, end, 0.0f, 0.0f, 0.8f, 0.0f, 0.0f, 0.8f);
    addGridLine(0.0f, 0.0f, start, 0.0f, 0.0f, end, 0.0f, 0.0f, 0.8f, 0.8f);
    addGridLine(0.0f, 0.0f, 0.0f, 0.0f, end/2, 0.0f, 0.0f, 0.8f, 0.0f, 0.8f);

    for (int i = -lineCount; i <= lineCount; i++) {
        float pos = i * gridSpacing;

        if (qAbs(pos) < 0.001f) continue;

        float distanceFromCenter = qAbs(i) / static_cast<float>(lineCount);
        float alpha = qMax(0.05f, 0.5f - 0.4f * distanceFromCenter);

        addGridLine(pos, 0.0f, start, pos, 0.0f, end, 0.5f, 0.5f, 0.5f, alpha);
        addGridLine(start, 0.0f, pos, end, 0.0f, pos, 0.5f, 0.5f, 0.5f, alpha);
    }

    glBindBuffer(GL_ARRAY_BUFFER, m_gridVBO);

    size_t vertSize = vertices.size() * sizeof(float);
    size_t colorSize = colors.size() * sizeof(float);

    glBufferData(GL_ARRAY_BUFFER, vertSize + colorSize, nullptr, GL_STATIC_DRAW);
    glBufferSubData(GL_ARRAY_BUFFER, 0, vertSize, vertices.data());
    glBufferSubData(GL_ARRAY_BUFFER, vertSize, colorSize, colors.data());

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_gridIBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(unsigned int), indices.data(), GL_STATIC_DRAW);

    m_gridVertexCount = vertices.size() / 3;
    m_gridIndexCount = indices.size();

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

    m_gridNeedsUpdate = false;
}

// Make sure to use the correct return type that matches your header declaration
OpenGLWidget::Mesh OpenGLWidget::processAssimpMesh(const aiMesh* mesh, const aiScene* scene)
{
    Mesh newMesh;

    // Extract mesh name if available
    if (mesh->mName.length > 0) {
        newMesh.name = QString::fromUtf8(mesh->mName.C_Str());
    }

    // Process vertices and normals
    for (unsigned int i = 0; i < mesh->mNumVertices; i++) {
        newMesh.vertices.emplace_back(
            mesh->mVertices[i].x,
            mesh->mVertices[i].y,
            mesh->mVertices[i].z
            );

        if (mesh->HasNormals()) {
            newMesh.normals.emplace_back(
                mesh->mNormals[i].x,
                mesh->mNormals[i].y,
                mesh->mNormals[i].z
                );
        }
    }

    // Process indices for triangular faces
    for (unsigned int i = 0; i < mesh->mNumFaces; i++) {
        aiFace face = mesh->mFaces[i];
        for (unsigned int j = 0; j < face.mNumIndices; j++) {
            newMesh.indices.push_back(face.mIndices[j]);
        }
    }

    // Extract material information if available
    if (mesh->mMaterialIndex >= 0 && scene->mMaterials) {
        aiMaterial* material = scene->mMaterials[mesh->mMaterialIndex];
        aiString matName;
        if (material->Get(AI_MATKEY_NAME, matName) == AI_SUCCESS) {
            newMesh.materialName = QString::fromUtf8(matName.C_Str());
        } else {
            newMesh.materialName = QString("Material_%1").arg(mesh->mMaterialIndex);
        }
    }

    newMesh.vertexCount = static_cast<int>(newMesh.vertices.size());
    newMesh.indexCount = static_cast<int>(newMesh.indices.size());

    return newMesh;
}

// Fixed processAssimpNode implementation with proper scoping for Mesh type

bool OpenGLWidget::processAssimpNode(const aiNode* node, const aiScene* scene, Model& model, int parentIndex)
{
    // Create a new model node
    ModelNode modelNode;
    modelNode.name = node->mName.length > 0 ?
                         QString::fromUtf8(node->mName.C_Str()) :
                         QString("Node_%1").arg(model.nodes.size());
    modelNode.parentIndex = parentIndex;

    // Extract local transformation
    aiVector3D scaling, rotation, position;
    node->mTransformation.Decompose(scaling, rotation, position);

    modelNode.position = QVector3D(position.x, position.y, position.z);
    modelNode.rotation = QVector3D(rotation.x * 180.0f / M_PI,
                                   rotation.y * 180.0f / M_PI,
                                   rotation.z * 180.0f / M_PI);
    modelNode.scale = QVector3D(scaling.x, scaling.y, scaling.z);
    modelNode.visible = true;
    modelNode.selected = false;

    // Process all meshes assigned to this node
    for (unsigned int i = 0; i < node->mNumMeshes; i++) {
        aiMesh* mesh = scene->mMeshes[node->mMeshes[i]];
        OpenGLWidget::Mesh newMesh = processAssimpMesh(mesh, scene);

        // Set mesh name - combine node name and mesh index if multiple meshes
        if (node->mNumMeshes > 1 && newMesh.name.isEmpty()) {
            newMesh.name = QString("%1_Mesh%2").arg(modelNode.name).arg(i);
        } else if (newMesh.name.isEmpty()) {
            newMesh.name = modelNode.name;
        }

        modelNode.meshes.push_back(newMesh);
    }

    // Store this node's index
    int nodeIndex = model.nodes.size();

    // Add the node to the model
    model.nodes.push_back(modelNode);

    // Process child nodes recursively
    for (unsigned int i = 0; i < node->mNumChildren; i++) {
        int childIndex = model.nodes.size();
        processAssimpNode(node->mChildren[i], scene, model, nodeIndex);
        model.nodes[nodeIndex].children.push_back(childIndex);
    }

    return true;
}


void OpenGLWidget::drawModelNode(Model& model, int nodeIndex)
{
    const ModelNode& node = model.nodes[nodeIndex];
    if (!node.visible) return;

    glPushMatrix();

    // Apply node's local transform
    glTranslatef(node.position.x(), node.position.y(), node.position.z());
    glRotatef(node.rotation.x(), 1.0f, 0.0f, 0.0f);
    glRotatef(node.rotation.y(), 0.0f, 1.0f, 0.0f);
    glRotatef(node.rotation.z(), 0.0f, 0.0f, 1.0f);
    glScalef(node.scale.x(), node.scale.y(), node.scale.z());

    // Draw all meshes for this node
    for (size_t i = 0; i < node.meshes.size(); i++) {
        // Create a non-const reference to allow initialization
        Mesh& meshRef = const_cast<Mesh&>(node.meshes[i]);

        if (!meshRef.vboInitialized) {
            initializeMeshVBO(meshRef);
        }

        if (meshRef.vboInitialized && meshRef.vertexCount > 0) {
            glEnableClientState(GL_VERTEX_ARRAY);
            glEnableClientState(GL_NORMAL_ARRAY);

            glBindBuffer(GL_ARRAY_BUFFER, meshRef.vbo);
            glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, meshRef.ibo);

            const int stride = 6 * sizeof(float);
            glVertexPointer(3, GL_FLOAT, stride, 0);
            glNormalPointer(GL_FLOAT, stride, (void*)(3 * sizeof(float)));

            // After drawing all meshes of the node (and possibly its children), add:
            if (node.selected) {
                glLineWidth(1.0f);
                glColor3f(1.0f, 1.0f, 0.0f);  // Yellow highlight color
                // Compute this node’s local bounding box (min and max in its local coords)
                QVector3D localMin(FLT_MAX, FLT_MAX, FLT_MAX);
                QVector3D localMax(-FLT_MAX, -FLT_MAX, -FLT_MAX);
                // Consider all vertices of all meshes in this node (and optionally children if you want the entire sub-tree)
                for (const auto &mesh : node.meshes) {
                    for (const QVector3D &v : mesh.vertices) {
                        localMin.setX(qMin(localMin.x(), v.x()));
                        localMin.setY(qMin(localMin.y(), v.y()));
                        localMin.setZ(qMin(localMin.z(), v.z()));
                        localMax.setX(qMax(localMax.x(), v.x()));
                        localMax.setY(qMax(localMax.y(), v.y()));
                        localMax.setZ(qMax(localMax.z(), v.z()));
                    }
                }
                // (Optional: If you want to include child nodes’ geometry in the highlight when a parent node is selected,
                // you can recursively include mesh vertices from all descendant nodes as well.)
                // Draw lines forming the bounding box, similar to model bounding box:
                float x1 = localMin.x(), x2 = localMax.x();
                float y1 = localMin.y(), y2 = localMax.y();
                float z1 = localMin.z(), z2 = localMax.z();
                glBegin(GL_LINES);
                // bottom face rectangle
                glVertex3f(x1,y1,z1); glVertex3f(x2,y1,z1);
                glVertex3f(x2,y1,z1); glVertex3f(x2,y1,z2);
                glVertex3f(x2,y1,z2); glVertex3f(x1,y1,z2);
                glVertex3f(x1,y1,z2); glVertex3f(x1,y1,z1);
                // top face rectangle
                glVertex3f(x1,y2,z1); glVertex3f(x2,y2,z1);
                glVertex3f(x2,y2,z1); glVertex3f(x2,y2,z2);
                glVertex3f(x2,y2,z2); glVertex3f(x1,y2,z2);
                glVertex3f(x1,y2,z2); glVertex3f(x1,y2,z1);
                // vertical edges
                glVertex3f(x1,y1,z1); glVertex3f(x1,y2,z1);
                glVertex3f(x2,y1,z1); glVertex3f(x2,y2,z1);
                glVertex3f(x2,y1,z2); glVertex3f(x2,y2,z2);
                glVertex3f(x1,y1,z2); glVertex3f(x1,y2,z2);
                glEnd();
            } else {
                glColor3f(0.8f, 0.8f, 0.8f); // Default color
            }

            glDrawElements(GL_TRIANGLES, meshRef.indexCount, GL_UNSIGNED_INT, 0);

            glDisableClientState(GL_NORMAL_ARRAY);
            glDisableClientState(GL_VERTEX_ARRAY);
            glBindBuffer(GL_ARRAY_BUFFER, 0);
            glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
        }
    }

    // Draw all child nodes recursively
    for (int childIndex : node.children) {
        drawModelNode(model, childIndex);
    }

    glPopMatrix();
}

// Updated drawModel function
void OpenGLWidget::drawModel(Model& model)
{
    if (!model.visible) return;

    glPushMatrix();

    // Apply model-level transforms
    QVector3D center = (model.min + model.max) * 0.5f;

    // 1. Translate to center
    glTranslatef(center.x(), center.y(), center.z());

    // 2. Apply model global transforms
    glTranslatef(model.position.x(), model.position.y(), model.position.z());
    glRotatef(model.rotation.x(), 1.0f, 0.0f, 0.0f);
    glRotatef(model.rotation.y(), 0.0f, 1.0f, 0.0f);
    glRotatef(model.rotation.z(), 0.0f, 0.0f, 1.0f);
    glScalef(model.scale.x(), model.scale.y(), model.scale.z());

    // 3. Apply model offset (for centering)
    glTranslatef(model.offset.x(), model.offset.y(), model.offset.z());

    // 4. Translate back from center
    glTranslatef(-center.x(), -center.y(), -center.z());

    // Draw each node recursively starting with root nodes
    for (size_t i = 0; i < model.nodes.size(); i++) {
        if (model.nodes[i].parentIndex == -1) {
            drawModelNode(model, i);
        }
    }

    // Draw bounding box if selected
    if (model.selected) {
        glLineWidth(1.0f);
        glColor3f(1.0f, 1.0f, 0.0f); // Yellow

        float x1 = model.min.x(), x2 = model.max.x();
        float y1 = model.min.y(), y2 = model.max.y();
        float z1 = model.min.z(), z2 = model.max.z();

        glBegin(GL_LINES);
        // Bottom face
        glVertex3f(x1, y1, z1); glVertex3f(x2, y1, z1);
        glVertex3f(x2, y1, z1); glVertex3f(x2, y1, z2);
        glVertex3f(x2, y1, z2); glVertex3f(x1, y1, z2);
        glVertex3f(x1, y1, z2); glVertex3f(x1, y1, z1);
        // Top face
        glVertex3f(x1, y2, z1); glVertex3f(x2, y2, z1);
        glVertex3f(x2, y2, z1); glVertex3f(x2, y2, z2);
        glVertex3f(x2, y2, z2); glVertex3f(x1, y2, z2);
        glVertex3f(x1, y2, z2); glVertex3f(x1, y2, z1);
        // Connecting edges
        glVertex3f(x1, y1, z1); glVertex3f(x1, y2, z1);
        glVertex3f(x2, y1, z1); glVertex3f(x2, y2, z1);
        glVertex3f(x2, y1, z2); glVertex3f(x2, y2, z2);
        glVertex3f(x1, y1, z2); glVertex3f(x1, y2, z2);
        glEnd();
    }

    glPopMatrix();
}


void OpenGLWidget::initializeMeshVBO(Mesh& mesh)
{
    if (mesh.vboInitialized || mesh.vertices.empty()) return;

    glGenBuffers(1, &mesh.vbo);
    glGenBuffers(1, &mesh.ibo);

    glBindBuffer(GL_ARRAY_BUFFER, mesh.vbo);
    std::vector<float> vertexData;
    vertexData.reserve(mesh.vertices.size() * 6);

    for (size_t i = 0; i < mesh.vertices.size(); ++i) {
        vertexData.push_back(mesh.vertices[i].x());
        vertexData.push_back(mesh.vertices[i].y());
        vertexData.push_back(mesh.vertices[i].z());

        if (i < mesh.normals.size()) {
            vertexData.push_back(mesh.normals[i].x());
            vertexData.push_back(mesh.normals[i].y());
            vertexData.push_back(mesh.normals[i].z());
        } else {
            vertexData.push_back(0.0f);
            vertexData.push_back(0.0f);
            vertexData.push_back(1.0f);
        }
    }

    glBufferData(GL_ARRAY_BUFFER, vertexData.size() * sizeof(float), vertexData.data(), GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, mesh.ibo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, mesh.indices.size() * sizeof(unsigned int), mesh.indices.data(), GL_STATIC_DRAW);

    mesh.vboInitialized = true;
    mesh.vertexCount = mesh.vertices.size();
    mesh.indexCount = mesh.indices.size();

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
}


void OpenGLWidget::setFrontView()
{
    // Reset rotation
    m_xRotation = 0.0f;
    m_yRotation = 0.0f;

    // Adjust zoom to fit the scene
    if (m_sceneBoundsValid) {
        m_zoom = 0.8f; // Set to a reasonable value that shows the entire scene
    }

    m_viewChanged = true;
    update();
}

void OpenGLWidget::setTopView()
{
    m_xRotation = 90.0f;
    m_yRotation = 0.0f;
    m_viewChanged = true;
    update();
}

void OpenGLWidget::setSideView()
{
    m_xRotation = 0.0f;
    m_yRotation = 90.0f;
    m_viewChanged = true;
    update();
}

void OpenGLWidget::setIsometricView()
{
    m_xRotation = 45.0f;
    m_yRotation = 45.0f;
    m_viewChanged = true;
    update();
}
// In openglwidget.cpp, around line 1581, fix:
void OpenGLWidget::setCenteredView()
{
    // Reset pan offset
    m_panOffset = QVector3D(0.0f, 0.0f, 0.0f);

    // Find selected point cloud or model
    bool foundSelected = false;
    QVector3D center(0.0f, 0.0f, 0.0f);

    // Check if we have a selected point cloud
    for (const auto& pc : m_pointClouds) {
        if (pc.selected && pc.visible) {
            center = (pc.min + pc.max) * 0.5f;
            foundSelected = true;
            break;
        }
    }

    // If no selected point cloud, check models
    if (!foundSelected) {
        for (const auto& model : m_models) {
            if (model.selected && model.visible) {
                center = (model.min + model.max) * 0.5f;
                foundSelected = true;
                break;
            }
        }
    }

    // If a selection was found, update scene center
    if (foundSelected) {
        // Make sure m_sceneCenter is QVector3D in the header file
        m_sceneCenter = center;
    }

    // Otherwise we'll use the existing scene bounds
    m_viewChanged = true;
    update();
}
void OpenGLWidget::drawDefaultTriangle()
{
    glTranslatef(m_panOffset.x(), m_panOffset.y(), 0.0f);
    glScalef(m_zoom, m_zoom, m_zoom);
    glRotatef(m_xRotation, 1.0f, 0.0f, 0.0f);
    glRotatef(m_yRotation, 0.0f, 1.0f, 0.0f);

    glBegin(GL_TRIANGLES);
    glColor3f(1.0f, 0.0f, 0.0f);
    glVertex2f(-0.5f, -0.5f);
    glColor3f(0.0f, 1.0f, 0.0f);
    glVertex2f(0.5f, -0.5f);
    glColor3f(0.0f, 0.0f, 1.0f);
    glVertex2f(0.0f, 0.5f);
    glEnd();
}

void OpenGLWidget::drawGrid()
{
    if (!m_showGrid || m_gridIndexCount == 0) return;

    glPushAttrib(GL_ENABLE_BIT | GL_CURRENT_BIT | GL_LINE_BIT);

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    glEnable(GL_LINE_SMOOTH);
    glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);

    glLineWidth(1.0f);

    glEnable(GL_POLYGON_OFFSET_FILL);
    glPolygonOffset(1.0f, 1.0f);

    glBindBuffer(GL_ARRAY_BUFFER, m_gridVBO);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_gridIBO);

    glEnableClientState(GL_VERTEX_ARRAY);
    glEnableClientState(GL_COLOR_ARRAY);

    glVertexPointer(3, GL_FLOAT, 0, 0);

    size_t colorOffset = m_gridVertexCount * 3 * sizeof(float);
    glColorPointer(4, GL_FLOAT, 0, (void*)colorOffset);

    glDrawElements(GL_LINES, m_gridIndexCount, GL_UNSIGNED_INT, 0);

    glDisableClientState(GL_COLOR_ARRAY);
    glDisableClientState(GL_VERTEX_ARRAY);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
    glDisable(GL_POLYGON_OFFSET_FILL);
    glPopAttrib();
}

void OpenGLWidget::drawCoordinateSystem()
{
    if (!m_showAxis || width() <= 0 || height() <= 0)
        return;
    if (!m_showAxis || width() <= 10 || height() <= 10) return;
    // Use per-widget image cache instead of static
    if (m_axisIndicator.isNull() || m_viewChanged ||
        m_axisIndicator.width() != width() || m_axisIndicator.height() != height()) {

        m_viewChanged = true;
        m_axisIndicator = QImage(width(), height(), QImage::Format_ARGB32_Premultiplied);
        m_axisIndicator.fill(Qt::transparent);

        QPainter painter(&m_axisIndicator);
        painter.setRenderHints(QPainter::Antialiasing | QPainter::TextAntialiasing);

        const int size = 80;
        const int margin = 20;
        const int centerX = width() - margin - size / 2;
        const int centerY = height() - margin - size / 2;
        const float axisLength = size * 0.35f;
        const float sphereRadius = 3.0f;

        painter.setOpacity(0.8);
        painter.fillRect(centerX - size / 2, centerY - size / 2, size, size, QColor(20, 20, 20));

        QMatrix4x4 rotMatrix;
        rotMatrix.rotate(-m_xRotation, 1.0f, 0.0f, 0.0f);
        rotMatrix.rotate(-m_yRotation, 0.0f, 1.0f, 0.0f);

        QVector3D xAxis = rotMatrix.map(QVector3D(axisLength, 0, 0));
        QVector3D yAxis = rotMatrix.map(QVector3D(0, axisLength, 0));
        QVector3D zAxis = rotMatrix.map(QVector3D(0, 0, axisLength));

        painter.setPen(QPen(Qt::red, 2));   painter.drawLine(centerX, centerY, centerX + xAxis.x(), centerY - xAxis.y());
        painter.setPen(QPen(Qt::green, 2)); painter.drawLine(centerX, centerY, centerX + yAxis.x(), centerY - yAxis.y());
        painter.setPen(QPen(Qt::blue, 2));  painter.drawLine(centerX, centerY, centerX + zAxis.x(), centerY - zAxis.y());

        painter.setPen(Qt::NoPen);
        painter.setBrush(Qt::red);   painter.drawEllipse(QPointF(centerX + xAxis.x(), centerY - xAxis.y()), sphereRadius, sphereRadius);
        painter.setBrush(Qt::green); painter.drawEllipse(QPointF(centerX + yAxis.x(), centerY - yAxis.y()), sphereRadius, sphereRadius);
        painter.setBrush(Qt::blue);  painter.drawEllipse(QPointF(centerX + zAxis.x(), centerY - zAxis.y()), sphereRadius, sphereRadius);

        QFont font = painter.font();
        font.setBold(true);
        font.setPointSize(10);
        painter.setFont(font);

        painter.setPen(Qt::red);   painter.drawText(QPoint(centerX + xAxis.x() + 5, centerY - xAxis.y() + 5), "X");
        painter.setPen(Qt::green); painter.drawText(QPoint(centerX + yAxis.x() + 5, centerY - yAxis.y() + 5), "Y");
        painter.setPen(Qt::blue);  painter.drawText(QPoint(centerX + zAxis.x() + 5, centerY - zAxis.y() + 5), "Z");
        painter.end();

        m_viewChanged = false; // Reset the viewChanged flag after update
    }

    if (!m_axisIndicator.isNull()) {
        glMatrixMode(GL_PROJECTION);
        glPushMatrix();
        glLoadIdentity();
        glOrtho(0, width(), height(), 0, -1, 1);

        glMatrixMode(GL_MODELVIEW);
        glPushMatrix();
        glLoadIdentity();

        glDisable(GL_DEPTH_TEST);
        glEnable(GL_TEXTURE_2D);
        GLuint texId;
        glGenTextures(1, &texId);
        glBindTexture(GL_TEXTURE_2D, texId);

        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, m_axisIndicator.width(), m_axisIndicator.height(),
                     0, GL_BGRA, GL_UNSIGNED_BYTE, m_axisIndicator.bits());

        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        glColor4f(1, 1, 1, 1);
        glBegin(GL_QUADS);
        glTexCoord2f(0, 0); glVertex2f(0, 0);
        glTexCoord2f(1, 0); glVertex2f(width(), 0);
        glTexCoord2f(1, 1); glVertex2f(width(), height());
        glTexCoord2f(0, 1); glVertex2f(0, height());
        glEnd();

        glBindTexture(GL_TEXTURE_2D, 0);
        glDeleteTextures(1, &texId);
        glDisable(GL_TEXTURE_2D);
        glDisable(GL_BLEND);
        glEnable(GL_DEPTH_TEST);

        glMatrixMode(GL_PROJECTION); glPopMatrix();
        glMatrixMode(GL_MODELVIEW); glPopMatrix();
    }
}


// Fix for the getSelectedItemsTransform method
QList<QVariant> OpenGLWidget::getSelectedItemsTransform() const
{
    QList<QVariant> result;

    // Return an empty list if nothing is selected
    bool hasSelection = false;

    // Check for selected point clouds
    for (const auto& pc : m_pointClouds) {
        if (pc.selected) {
            hasSelection = true;

            // Get center point of the point cloud as position
            QVector3D center = pc.boundingBox(); // Using our new method
            result.append(QVariant::fromValue(center + pc.offset)); // Add offset

            // Return rotation
            result.append(QVariant::fromValue(pc.rotation));

            // Return scale
            result.append(QVariant::fromValue(pc.scale));

            // Return after first selected item for simplicity
            return result;
        }
    }

    // Check for selected models
    for (const auto& model : m_models) {
        if (model.selected) {
            hasSelection = true;

            // Return model transform values
            result.append(QVariant::fromValue(model.position));
            result.append(QVariant::fromValue(model.rotation));
            result.append(QVariant::fromValue(model.scale));

            // Return after first selected item for simplicity
            return result;
        }
    }

    // If we get here, nothing was selected
    return result;
}

// Fix for the applyTransformToSelection method
bool OpenGLWidget::applyTransformToSelection(const QVector3D& position, const QVector3D& rotation, const QVector3D& scale)
{
    bool anythingModified = false;

    // Apply to point clouds
    for (auto& pc : m_pointClouds) {
        if (pc.selected) {
            // Calculate center point
            QVector3D center = pc.boundingBox();

            // Set the offset (which is position - center)
            pc.offset = position - center;

            // Set rotation and scale directly
            pc.rotation = rotation;
            pc.scale = scale;

            anythingModified = true;
        }
    }

    // Apply to models
    for (auto& model : m_models) {
        if (model.selected) {
            // Update model transform
            model.position = position;
            model.rotation = rotation;
            model.scale = scale;

            anythingModified = true;
        }
    }

    return anythingModified;
}


void OpenGLWidget::add2DGrid(int rows, int columns, float cellSize, const QString& name)
{
    Grid2D grid;
    grid.rows = rows;
    grid.columns = columns;
    grid.cellSize = cellSize;
    grid.name = name;
    grid.visible = true;
    grid.selected = false;
    grid.height = 0.0f;

    // Calculate vertices for bounding box
    std::vector<QVector3D> vertices;
    float width = columns * cellSize;
    float depth = rows * cellSize;
    vertices.push_back(QVector3D(-width/2.0f, 0.0f, -depth/2.0f));
    vertices.push_back(QVector3D(width/2.0f, 0.0f, -depth/2.0f));
    vertices.push_back(QVector3D(width/2.0f, 0.0f, depth/2.0f));
    vertices.push_back(QVector3D(-width/2.0f, 0.0f, depth/2.0f));
    calculateBoundingBox(vertices, grid.min, grid.max);

    m_grids2D.push_back(grid);
    m_sceneBoundsValid = false;
    qDebug() << "2D Grid '" << name << "' added with" << rows << "rows and" << columns << "columns";
    update();
}

const std::vector<OpenGLWidget::Grid2D>& OpenGLWidget::get2DGrids() const {
    return m_grids2D;
}

const std::vector<OpenGLWidget::Grid2D>& OpenGLWidget::getGrids2D() const {
    return m_grids2D;
}

// New method: clear2DGrids
void OpenGLWidget::clear2DGrids()
{
    makeCurrent();
    for (auto& grid : m_grids2D) {
        if (grid.vboInitialized) {
            glDeleteBuffers(1, &grid.vbo);
            glDeleteBuffers(1, &grid.ibo);
        }
    }
    m_grids2D.clear();
    m_sceneBoundsValid = false;
    update();
    doneCurrent();
}



// New method: set2DGridVisibility
void OpenGLWidget::set2DGridVisibility(const QString& name, bool visible)
{
    for (auto& grid : m_grids2D) {
        if (grid.name == name) {
            grid.visible = visible;
            m_sceneBoundsValid = false;
            updateObjectVisibilityBetweenGrids();
            update();
            return;
        }
    }
    qDebug() << "2D Grid with name '" << name << "' not found.";
}

// New method: is2DGridVisible
bool OpenGLWidget::is2DGridVisible(const QString& name) const
{
    for (const auto& grid : m_grids2D) {
        if (grid.name == name) {
            return grid.visible;
        }
    }
    return false;
}

// New method: set2DGridSelected
void OpenGLWidget::set2DGridSelected(const QString& name, bool selected)
{
    for (auto& grid : m_grids2D) {
        if (grid.name == name) {
            grid.selected = selected;
            update();
            return;
        }
    }
    qDebug() << "2D Grid with name '" << name << "' not found.";
}

// New method: is2DGridSelected
bool OpenGLWidget::is2DGridSelected(const QString& name) const
{
    for (const auto& grid : m_grids2D) {
        if (grid.name == name) {
            return grid.selected;
        }
    }
    return false;
}

void OpenGLWidget::initialize2DGridVBO(Grid2D& grid)
{
    if (grid.vboInitialized || grid.rows <= 0 || grid.columns <= 0) return;

    // Generate buffer objects
    glGenBuffers(1, &grid.vbo);
    glGenBuffers(1, &grid.ibo);

    std::vector<float> vertices;
    std::vector<unsigned int> indices;

    float width = grid.columns * grid.cellSize;
    float depth = grid.rows * grid.cellSize;
    float startX = -width / 2.0f;
    float startZ = -depth / 2.0f;

    // Generate vertices
    for (int i = 0; i <= grid.rows; ++i) {
        for (int j = 0; j <= grid.columns; ++j) {
            vertices.push_back(startX + j * grid.cellSize); // X
            vertices.push_back(0.0f);                       // Y (on XZ plane)
            vertices.push_back(startZ + i * grid.cellSize); // Z
        }
    }

    // Generate indices for horizontal lines
    for (int i = 0; i <= grid.rows; ++i) {
        // One continuous line per row
        for (int j = 0; j < grid.columns; ++j) {
            unsigned int currentPoint = i * (grid.columns + 1) + j;
            indices.push_back(currentPoint);
            indices.push_back(currentPoint + 1);
        }
    }

    // Generate indices for vertical lines
    for (int j = 0; j <= grid.columns; ++j) {
        // One continuous line per column
        for (int i = 0; i < grid.rows; ++i) {
            unsigned int currentPoint = i * (grid.columns + 1) + j;
            indices.push_back(currentPoint);
            indices.push_back(currentPoint + (grid.columns + 1));
        }
    }

    // Bind and upload vertex data
    glBindBuffer(GL_ARRAY_BUFFER, grid.vbo);
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(float), vertices.data(), GL_STATIC_DRAW);

    // Bind and upload index data
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, grid.ibo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(unsigned int), indices.data(), GL_STATIC_DRAW);

    grid.vertexCount = vertices.size() / 3;
    grid.indexCount = indices.size();
    grid.vboInitialized = true;

    // Unbind buffers
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
}
// Add these to the Model struct in openglwidget.h
struct ModelNode {
    QString name;
    std::vector<OpenGLWidget::Mesh> meshes;  // Meshes directly attached to this node
    std::vector<int> children; // Indices of child nodes in the model's nodes vector
    int parentIndex;           // Index of parent node (-1 for root)
    QVector3D position;        // Local position
    QVector3D rotation;        // Local rotation
    QVector3D scale;           // Local scale
    bool visible;              // Visibility state

    ModelNode() :
        parentIndex(-1),
        position(0.0f, 0.0f, 0.0f),
        rotation(0.0f, 0.0f, 0.0f),
        scale(1.0f, 1.0f, 1.0f),
        visible(true) {}
};
// Updated Model struct
struct Model {
    QString name;
    std::vector<ModelNode> nodes;    // Hierarchy of nodes
    QVector3D min, max;              // Overall bounding box
    bool visible;
    bool selected;

    // Transform for the entire model
    QVector3D position;
    QVector3D rotation;
    QVector3D scale;
    QVector3D offset;

    // Constructor with defaults
    Model() :
        visible(true),
        selected(false),
        position(0.0f, 0.0f, 0.0f),
        rotation(0.0f, 0.0f, 0.0f),
        scale(1.0f, 1.0f, 1.0f),
        offset(0.0f, 0.0f, 0.0f) {}
};
// Update the Mesh struct to include name and material info
struct Mesh {
    QString name;           // Mesh name
    QString materialName;   // Material name
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
};
// New method: draw2DGrid
void OpenGLWidget::draw2DGrid(Grid2D& grid)
{
    // Skip invisible grids
    if (!grid.visible) return;

    // Initialize VBO if not already done
    if (!grid.vboInitialized) {
        initialize2DGridVBO(grid);
    }

    if (grid.vertexCount == 0 || grid.indexCount == 0) return;

    // Save the current OpenGL state
    glPushAttrib(GL_ENABLE_BIT | GL_CURRENT_BIT | GL_LINE_BIT | GL_DEPTH_BUFFER_BIT);

    // Enable blending for smoother lines
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // Enable depth testing but with equal function to prevent z-fighting
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LEQUAL);

    // Enable antialiasing for lines
    glEnable(GL_LINE_SMOOTH);
    glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);

    // Set line properties
    glLineWidth(1.0f);
    glColor4f(1.0f, 1.0f, 1.0f, 1.0f); // Bright white with full opacity

    // Set up vertex arrays
    glEnableClientState(GL_VERTEX_ARRAY);

    // Bind buffer objects
    glBindBuffer(GL_ARRAY_BUFFER, grid.vbo);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, grid.ibo);

    // Define vertex attribute pointers
    glVertexPointer(3, GL_FLOAT, 0, nullptr);

    // Draw grid lines
    glDrawElements(GL_LINES, grid.indexCount, GL_UNSIGNED_INT, nullptr);

    // Cleanup
    glDisableClientState(GL_VERTEX_ARRAY);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

    // Draw bounding box if selected
    if (grid.selected) {
        glLineWidth(2.0f);
        glColor4f(1.0f, 1.0f, 0.0f, 1.0f); // Yellow with full opacity

        float x1 = grid.min.x(), x2 = grid.max.x();
        float y1 = grid.min.y(), y2 = grid.max.y();
        float z1 = grid.min.z(), z2 = grid.max.z();

        glBegin(GL_LINE_LOOP);
        // Bottom face at y=y1
        glVertex3f(x1, y1, z1);
        glVertex3f(x2, y1, z1);
        glVertex3f(x2, y1, z2);
        glVertex3f(x1, y1, z2);
        glEnd();
    }

    // Restore the previous OpenGL state
    glPopAttrib();
}

bool OpenGLWidget::getPointCloudByName(const QString &name, std::vector<QVector3D> &points, std::vector<QVector3D> &colors)
{
    for (const auto &pc : m_pointClouds) {
        if (pc.name == name) {
            points = pc.points;
            colors = pc.colors;
            return true;
        }
    }
    return false;
}




////grid segment

void OpenGLWidget::setGridCellVisibility(const QString& gridName, int row, int col, bool visible)
{
    qDebug() << "setGridCellVisibility: Grid:" << gridName << "Row:" << row << "Col:" << col << "Visible:" << visible;

    for (auto& grid : m_grids2D) {
        if (grid.name != gridName) {
            continue;
        }

        if (row < 0 || row >= grid.rows || col < 0 || col >= grid.columns) {
            qDebug() << "setGridCellVisibility: Invalid row or column for grid:" << gridName;
            return;
        }

        // Generate unique name for the point cloud section
        QString sectionName = QString("%1_%2_%3").arg(gridName).arg(row).arg(col);

        // Check if the section already exists
        auto it = std::find_if(m_pointClouds.begin(), m_pointClouds.end(),
                               [sectionName](const PointCloud& pc) { return pc.name == sectionName; });

        float cellXMin = grid.min.x() + col * grid.cellSize;
        float cellXMax = cellXMin + grid.cellSize;
        float cellZMin = grid.min.z() + row * grid.cellSize;
        float cellZMax = cellZMin + grid.cellSize;

        if (visible) {
            // When setting to visible, we want to show/create the section
            if (it != m_pointClouds.end()) {
                // Section exists, update its visibility
                it->visible = true;
                qDebug() << "Updated visibility for point cloud section:" << sectionName << "to visible";
            } else {
                // Section doesn't exist, create it
                createPointCloudSection(grid, gridName, row, col, sectionName);
            }

            // Remove points from other point clouds within this cell when making section visible
            removePointsFromCell(cellXMin, cellXMax, cellZMin, cellZMax, sectionName);
        } else {
            // When setting to invisible, we want to hide the section but keep points in other clouds
            if (it != m_pointClouds.end()) {
                // Section exists, hide it
                it->visible = false;
                qDebug() << "Updated visibility for point cloud section:" << sectionName << "to hidden";
            }

            // When hiding the section, we DON'T remove points from other clouds
            // This reverses the previous behavior to match what you want
        }

        m_sceneBoundsValid = false;
        update();
        return; // Grid was found, we're done
    }

    qDebug() << "setGridCellVisibility: Grid not found:" << gridName;
}
void OpenGLWidget::removePointsFromCell(float xMin, float xMax, float zMin, float zMax, const QString& excludeSectionName)
{
    bool changed = false;

    makeCurrent();

    // Process each point cloud
    for (auto& pc : m_pointClouds) {
        // Skip the section we're explicitly excluding (if any)
        if (pc.name == excludeSectionName) {
            continue;
        }

        // Skip sections that are already not visible or are grid cell sections
        if (!pc.visible || !pc.parentName.isEmpty()) {
            continue;
        }

        // Remove points that fall in this cell
        std::vector<QVector3D> newPoints;
        std::vector<QVector3D> newColors;

        for (size_t i = 0; i < pc.points.size(); ++i) {
            float x = pc.points[i].x();
            float z = pc.points[i].z();

            // Keep points outside the cell bounds
            if (x < xMin || x >= xMax || z < zMin || z >= zMax) {
                newPoints.push_back(pc.points[i]);
                if (i < pc.colors.size()) {
                    newColors.push_back(pc.colors[i]);
                }
            } else {
                changed = true; // A point was removed
            }
        }

        // Update the point cloud if points were removed
        if (pc.points.size() != newPoints.size()) {
            pc.points = newPoints;
            pc.colors = newColors;
            pc.pointCount = newPoints.size();

            // Recalculate bounding box if there are still points
            if (!newPoints.empty()) {
                calculateBoundingBox(newPoints, pc.min, pc.max);
            }

            // Reinitialize VBO with new points
            if (pc.vboInitialized) {
                updatePointCloudVBO(pc);
            }

            qDebug() << "Removed" << (pc.points.size() - newPoints.size()) << "points from" << pc.name
                     << "in cell bounds: X(" << xMin << "," << xMax << ") Z(" << zMin << "," << zMax << ")";
        }
    }

    if (changed) {
        m_sceneBoundsValid = false;
        update();
    }

    doneCurrent();
}
void OpenGLWidget::createPointCloudSection(const Grid2D& grid, const QString& gridName, int row, int col, const QString& sectionName)
{
    float cellXMin = grid.min.x() + col * grid.cellSize;
    float cellXMax = cellXMin + grid.cellSize;
    float cellZMin = grid.min.z() + row * grid.cellSize;
    float cellZMax = cellZMin + grid.cellSize;

    std::vector<QVector3D> sectionPoints;
    std::vector<QVector3D> sectionColors;

    // Check if this section already exists
    auto existingSection = std::find_if(m_pointClouds.begin(), m_pointClouds.end(),
                                        [sectionName](const PointCloud& pc) { return pc.name == sectionName; });

    if (existingSection != m_pointClouds.end()) {
        // If it exists, update its visibility
        existingSection->visible = true;
        qDebug() << "Section already exists, updating visibility:" << sectionName;
        m_sceneBoundsValid = false;
        update();
        return;
    }

    // Find points within the grid cell from visible point clouds
    for (const auto& pc : m_pointClouds) {
        if (!pc.visible || !pc.parentName.isEmpty()) {
            continue; // Skip invisible or already split sections
        }

        for (size_t i = 0; i < pc.points.size(); ++i) {
            float x = pc.points[i].x();
            float z = pc.points[i].z();
            if (x >= cellXMin && x < cellXMax && z >= cellZMin && z < cellZMax) {
                sectionPoints.push_back(pc.points[i]);
                sectionColors.push_back(i < pc.colors.size() ? pc.colors[i] : QVector3D(1.0f, 1.0f, 1.0f));
            }
        }
    }

    if (!sectionPoints.empty()) {
        PointCloud sectionPC;
        sectionPC.name = sectionName;
        sectionPC.points = sectionPoints;
        sectionPC.colors = sectionColors;
        sectionPC.pointCount = sectionPoints.size();
        sectionPC.visible = true;
        sectionPC.selected = false;
        sectionPC.vboInitialized = false;
        sectionPC.parentName = gridName; // Mark as a section
        calculateBoundingBox(sectionPoints, sectionPC.min, sectionPC.max);

        makeCurrent();
        initializePointCloudVBO(sectionPC);
        m_pointClouds.push_back(sectionPC);
        doneCurrent();

        // Remove these points from other point clouds
        removePointsFromCell(cellXMin, cellXMax, cellZMin, cellZMax, sectionName);

        qDebug() << "Created new point cloud section:" << sectionName << "with" << sectionPoints.size() << "points";
        m_sceneBoundsValid = false;
        update();
    } else {
        qDebug() << "No points found in grid cell to create section:" << sectionName;
    }
}
void OpenGLWidget::clearGridCellSelections(const QString& gridName)
{
    bool changed = false;

    makeCurrent();

    // Find the grid to get its properties
    Grid2D* targetGrid = nullptr;
    for (auto& grid : m_grids2D) {
        if (grid.name == gridName) {
            targetGrid = &grid;
            break;
        }
    }

    if (!targetGrid) {
        qDebug() << "clearGridCellSelections: Grid not found:" << gridName;
        doneCurrent();
        return;
    }

    // Hide all sections of this grid but DON'T remove points from cells
    for (auto& pc : m_pointClouds) {
        if (!pc.parentName.isEmpty() && pc.parentName == gridName) {
            if (pc.visible) {
                pc.visible = false;
                changed = true;
                qDebug() << "Cleared visibility for point cloud section:" << pc.name;
            }
        }
    }

    if (changed) {
        m_sceneBoundsValid = false;
        update();
    }

    doneCurrent();
}
void OpenGLWidget::updatePointCloudVBO(PointCloud& pc)
{
    if (!pc.vboInitialized) {
        initializePointCloudVBO(pc);
        return;
    }

    // Delete old VBO
    if (pc.vbo) {
        glDeleteBuffers(1, &pc.vbo);
        pc.vbo = 0;
        pc.vboInitialized = false;
    }

    // Create new VBO with updated data
    glGenBuffers(1, &pc.vbo);
    glBindBuffer(GL_ARRAY_BUFFER, pc.vbo);

    std::vector<float> vertexData;
    vertexData.reserve(pc.points.size() * 6);

    for (size_t i = 0; i < pc.points.size(); ++i) {
        vertexData.push_back(pc.points[i].x());
        vertexData.push_back(pc.points[i].y());
        vertexData.push_back(pc.points[i].z());

        if (i < pc.colors.size()) {
            vertexData.push_back(pc.colors[i].x());
            vertexData.push_back(pc.colors[i].y());
            vertexData.push_back(pc.colors[i].z());
        } else {
            vertexData.push_back(1.0f);
            vertexData.push_back(1.0f);
            vertexData.push_back(1.0f);
        }
    }

    glBufferData(GL_ARRAY_BUFFER, vertexData.size() * sizeof(float), vertexData.data(), GL_STATIC_DRAW);
    pc.vboInitialized = true;
    pc.pointCount = pc.points.size();

    glBindBuffer(GL_ARRAY_BUFFER, 0);
}

void OpenGLWidget::add2DGridAtHeight(int rows, int columns, float cellSize, const QString& name, float height)
{
    makeCurrent();

    Grid2D grid;
    grid.rows = rows;
    grid.columns = columns;
    grid.cellSize = cellSize;
    grid.name = name;
    grid.visible = true;
    grid.selected = false;
    grid.height = height; // Set specified height

    // Generate buffer objects
    glGenBuffers(1, &grid.vbo);
    glGenBuffers(1, &grid.ibo);

    // Calculate vertices and indices for grid lines
    std::vector<float> vertices;
    std::vector<unsigned int> indices;

    float width = columns * cellSize;
    float depth = rows * cellSize;
    float startX = -width / 2.0f;
    float startZ = -depth / 2.0f;

    // Generate vertices
    for (int i = 0; i <= rows; ++i) {
        for (int j = 0; j <= columns; ++j) {
            vertices.push_back(startX + j * cellSize); // X
            vertices.push_back(height);                 // Y
            vertices.push_back(startZ + i * cellSize); // Z
        }
    }

    // Generate indices for horizontal lines
    for (int i = 0; i <= rows; ++i) {
        for (int j = 0; j < columns; ++j) {
            unsigned int currentPoint = i * (columns + 1) + j;
            indices.push_back(currentPoint);
            indices.push_back(currentPoint + 1);
        }
    }

    // Generate indices for vertical lines
    for (int j = 0; j <= columns; ++j) {
        for (int i = 0; i < rows; ++i) {
            unsigned int currentPoint = i * (columns + 1) + j;
            indices.push_back(currentPoint);
            indices.push_back(currentPoint + (columns + 1));
        }
    }

    // Bind and upload vertex data
    glBindBuffer(GL_ARRAY_BUFFER, grid.vbo);
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(float), vertices.data(), GL_STATIC_DRAW);

    // Bind and upload index data
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, grid.ibo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(unsigned int), indices.data(), GL_STATIC_DRAW);

    grid.vertexCount = vertices.size() / 3;
    grid.indexCount = indices.size();
    grid.vboInitialized = true;

    // Unbind buffers
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

    // Calculate bounding box
    std::vector<QVector3D> boundingVertices;
    boundingVertices.push_back(QVector3D(-width/2.0f, height, -depth/2.0f));
    boundingVertices.push_back(QVector3D(width/2.0f, height, -depth/2.0f));
    boundingVertices.push_back(QVector3D(width/2.0f, height, depth/2.0f));
    boundingVertices.push_back(QVector3D(-width/2.0f, height, depth/2.0f));
    calculateBoundingBox(boundingVertices, grid.min, grid.max);

    m_grids2D.push_back(grid);
    m_sceneBoundsValid = false;
    qDebug() << "2D Grid '" << name << "' added at height" << height << "with" << rows << "rows and" << columns << "columns";
    update();

    doneCurrent();
}
void OpenGLWidget::handleMultiGridParameters(int rows, int columns, float cellSize, int duplicateCount, const std::vector<float>& heights, const QString& baseName)
{
    if (duplicateCount != static_cast<int>(heights.size())) {
        qDebug() << "Mismatch between duplicate count" << duplicateCount << "and heights count" << heights.size();
        return;
    }

    makeCurrent();

    // Clear existing 2D grids with the same base name to avoid duplicates
    auto it = m_grids2D.begin();
    while (it != m_grids2D.end()) {
        if (it->name.startsWith(baseName)) {
            if (it->vboInitialized) {
                glDeleteBuffers(1, &it->vbo);
                glDeleteBuffers(1, &it->ibo);
            }
            it = m_grids2D.erase(it);
        } else {
            ++it;
        }
    }

    // Add new grids at specified heights
    for (int i = 0; i < duplicateCount; ++i) {
        QString gridName = QString("%1_Height%2").arg(baseName).arg(heights[i]);
        add2DGridAtHeight(rows, columns, cellSize, gridName, heights[i]);
    }

    m_sceneBoundsValid = false;
    update();

    doneCurrent();
}
void OpenGLWidget::add3DGrid(int rows, int columns, int layers, float cellSize, const QString& name)
{
    Grid3D grid;
    grid.rows = rows;
    grid.columns = columns;
    grid.layers = layers;
    grid.cellSize = cellSize;
    grid.name = name;
    grid.visible = true;
    grid.selected = false;

    // Calculate vertices for bounding box
    std::vector<QVector3D> vertices;
    float width = columns * cellSize;
    float depth = rows * cellSize;
    float height = layers * cellSize;
    vertices.push_back(QVector3D(-width/2.0f, -height/2.0f, -depth/2.0f));
    vertices.push_back(QVector3D(width/2.0f, -height/2.0f, -depth/2.0f));
    vertices.push_back(QVector3D(width/2.0f, -height/2.0f, depth/2.0f));
    vertices.push_back(QVector3D(-width/2.0f, -height/2.0f, depth/2.0f));
    vertices.push_back(QVector3D(-width/2.0f, height/2.0f, -depth/2.0f));
    vertices.push_back(QVector3D(width/2.0f, height/2.0f, -depth/2.0f));
    vertices.push_back(QVector3D(width/2.0f, height/2.0f, depth/2.0f));
    vertices.push_back(QVector3D(-width/2.0f, height/2.0f, depth/2.0f));
    calculateBoundingBox(vertices, grid.min, grid.max);

    m_grids3D.push_back(grid);
    m_sceneBoundsValid = false;
    qDebug() << "3D Grid '" << name << "' added with" << rows << "rows," << columns << "columns, and" << layers << "layers";
    update();
}
void OpenGLWidget::clear3DGrids()
{
    makeCurrent();
    for (auto& grid : m_grids3D) {
        if (grid.vboInitialized) {
            glDeleteBuffers(1, &grid.vbo);
            glDeleteBuffers(1, &grid.ibo);
        }
    }
    m_grids3D.clear();
    m_sceneBoundsValid = false;
    update();
    doneCurrent();
}
void OpenGLWidget::set3DGridVisibility(const QString& name, bool visible)
{
    for (auto& grid : m_grids3D) {
        if (grid.name == name) {
            grid.visible = visible;
            m_sceneBoundsValid = false;
            update();
            return;
        }
    }
    qDebug() << "3D Grid with name '" << name << "' not found.";
}
bool OpenGLWidget::is3DGridVisible(const QString& name) const
{
    for (const auto& grid : m_grids3D) {
        if (grid.name == name) {
            return grid.visible;
        }
    }
    return false;
}
void OpenGLWidget::set3DGridSelected(const QString& name, bool selected)
{
    for (auto& grid : m_grids3D) {
        if (grid.name == name) {
            grid.selected = selected;
            update();
            return;
        }
    }
    qDebug() << "3D Grid with name '" << name << "' not found.";
}
bool OpenGLWidget::is3DGridSelected(const QString& name) const
{
    for (const auto& grid : m_grids3D) {
        if (grid.name == name) {
            return grid.selected;
        }
    }
    return false;
}
void OpenGLWidget::initialize3DGridVBO(Grid3D& grid)
{
    if (grid.vboInitialized || grid.rows <= 0 || grid.columns <= 0 || grid.layers <= 0) return;

    // Generate buffer objects
    glGenBuffers(1, &grid.vbo);
    glGenBuffers(1, &grid.ibo);

    std::vector<float> vertices;
    std::vector<unsigned int> indices;

    float width = grid.columns * grid.cellSize;
    float depth = grid.rows * grid.cellSize;
    float height = grid.layers * grid.cellSize;
    float startX = -width / 2.0f;
    float startY = -height / 2.0f;
    float startZ = -depth / 2.0f;

    // Generate vertices
    for (int k = 0; k <= grid.layers; ++k) {
        for (int i = 0; i <= grid.rows; ++i) {
            for (int j = 0; j <= grid.columns; ++j) {
                vertices.push_back(startX + j * grid.cellSize); // X
                vertices.push_back(startY + k * grid.cellSize); // Y
                vertices.push_back(startZ + i * grid.cellSize); // Z
            }
        }
    }

    // Generate indices for lines along X (for each row and layer)
    for (int k = 0; k <= grid.layers; ++k) {
        for (int i = 0; i <= grid.rows; ++i) {
            for (int j = 0; j < grid.columns; ++j) {
                unsigned int base = k * (grid.rows + 1) * (grid.columns + 1) + i * (grid.columns + 1) + j;
                indices.push_back(base);
                indices.push_back(base + 1);
            }
        }
    }

    // Generate indices for lines along Z (for each column and layer)
    for (int k = 0; k <= grid.layers; ++k) {
        for (int j = 0; j <= grid.columns; ++j) {
            for (int i = 0; i < grid.rows; ++i) {
                unsigned int base = k * (grid.rows + 1) * (grid.columns + 1) + i * (grid.columns + 1) + j;
                indices.push_back(base);
                indices.push_back(base + (grid.columns + 1));
            }
        }
    }

    // Generate indices for lines along Y (for each row and column)
    for (int i = 0; i <= grid.rows; ++i) {
        for (int j = 0; j <= grid.columns; ++j) {
            for (int k = 0; k < grid.layers; ++k) {
                unsigned int base = k * (grid.rows + 1) * (grid.columns + 1) + i * (grid.columns + 1) + j;
                indices.push_back(base);
                indices.push_back(base + (grid.rows + 1) * (grid.columns + 1));
            }
        }
    }

    // Bind and upload vertex data
    glBindBuffer(GL_ARRAY_BUFFER, grid.vbo);
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(float), vertices.data(), GL_STATIC_DRAW);

    // Bind and upload index data
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, grid.ibo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(unsigned int), indices.data(), GL_STATIC_DRAW);

    grid.vertexCount = vertices.size() / 3;
    grid.indexCount = indices.size();
    grid.vboInitialized = true;

    // Unbind buffers
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
}
void OpenGLWidget::draw3DGrid(Grid3D& grid)
{
    // Skip invisible grids
    if (!grid.visible) return;

    // Initialize VBO if not already done
    if (!grid.vboInitialized) {
        initialize3DGridVBO(grid);
    }

    if (grid.vertexCount == 0 || grid.indexCount == 0) return;

    // Save the current OpenGL state
    glPushAttrib(GL_ENABLE_BIT | GL_CURRENT_BIT | GL_LINE_BIT | GL_DEPTH_BUFFER_BIT);

    // Enable blending for smoother lines
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // Enable depth testing but with equal function to prevent z-fighting
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LEQUAL);

    // Enable antialiasing for lines
    glEnable(GL_LINE_SMOOTH);
    glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);

    // Set line properties
    glLineWidth(1.0f);
    glColor4f(1.0f, 1.0f, 1.0f, 1.0f); // Bright white with full opacity

    // Set up vertex arrays
    glEnableClientState(GL_VERTEX_ARRAY);

    // Bind buffer objects
    glBindBuffer(GL_ARRAY_BUFFER, grid.vbo);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, grid.ibo);

    // Define vertex attribute pointers
    glVertexPointer(3, GL_FLOAT, 0, nullptr);

    // Draw grid lines
    glDrawElements(GL_LINES, grid.indexCount, GL_UNSIGNED_INT, nullptr);

    // Cleanup
    glDisableClientState(GL_VERTEX_ARRAY);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

    // Draw bounding box if selected
    if (grid.selected) {
        glLineWidth(2.0f);
        glColor4f(1.0f, 1.0f, 0.0f, 1.0f); // Yellow with full opacity

        float x1 = grid.min.x(), x2 = grid.max.x();
        float y1 = grid.min.y(), y2 = grid.max.y();
        float z1 = grid.min.z(), z2 = grid.max.z();

        glBegin(GL_LINES);
        // Bottom face
        glVertex3f(x1, y1, z1); glVertex3f(x2, y1, z1);
        glVertex3f(x2, y1, z1); glVertex3f(x2, y1, z2);
        glVertex3f(x2, y1, z2); glVertex3f(x1, y1, z2);
        glVertex3f(x1, y1, z2); glVertex3f(x1, y1, z1);
        // Top face
        glVertex3f(x1, y2, z1); glVertex3f(x2, y2, z1);
        glVertex3f(x2, y2, z1); glVertex3f(x2, y2, z2);
        glVertex3f(x2, y2, z2); glVertex3f(x1, y2, z2);
        glVertex3f(x1, y2, z2); glVertex3f(x1, y2, z1);
        // Connecting edges
        glVertex3f(x1, y1, z1); glVertex3f(x1, y2, z1);
        glVertex3f(x2, y1, z1); glVertex3f(x2, y2, z1);
        glVertex3f(x2, y1, z2); glVertex3f(x2, y2, z2);
        glVertex3f(x1, y1, z2); glVertex3f(x1, y2, z2);
        glEnd();
    }

    // Restore the previous OpenGL state
    glPopAttrib();
}

void OpenGLWidget::updateObjectVisibilityBetweenGrids()
{
    makeCurrent();

    // Step 1: Reset visibility for all split objects
    for (auto& pc : m_pointClouds) {
        if (!pc.parentName.isEmpty()) {
            // For split objects, determine if they should be visible based on grid visibility
            if (pc.name.endsWith("_Between")) {
                // Find which grids this section is between
                float sectionLowerY = pc.min.y();
                float sectionUpperY = pc.max.y();

                // Check if any grid in this range is invisible
                bool shouldBeHidden = false;
                for (const auto& grid : m_grids2D) {
                    if (grid.height >= sectionLowerY && grid.height <= sectionUpperY && !grid.visible) {
                        shouldBeHidden = true;
                        break;
                    }
                }

                pc.visible = !shouldBeHidden;
            }
        } else {
            // Restore original visibility for parent objects
            pc.visible = pc.originalVisibility;
        }
    }

    for (auto& model : m_models) {
        if (!model.parentName.isEmpty()) {
            // For split objects, determine if they should be visible based on grid visibility
            if (model.name.endsWith("_Between")) {
                // Find which grids this section is between
                float sectionLowerY = model.min.y();
                float sectionUpperY = model.max.y();

                // Check if any grid in this range is invisible
                bool shouldBeHidden = false;
                for (const auto& grid : m_grids2D) {
                    if (grid.height >= sectionLowerY && grid.height <= sectionUpperY && !grid.visible) {
                        shouldBeHidden = true;
                        break;
                    }
                }

                model.visible = !shouldBeHidden;
            }
        } else {
            // Restore original visibility for parent objects
            model.visible = model.originalVisibility;
        }
    }

    // Step 2: Delete all existing split objects
    auto pcIt = m_pointClouds.begin();
    while (pcIt != m_pointClouds.end()) {
        if (!pcIt->parentName.isEmpty()) {
            if (pcIt->vboInitialized) {
                glDeleteBuffers(1, &pcIt->vbo);
            }
            pcIt = m_pointClouds.erase(pcIt);
        } else {
            ++pcIt;
        }
    }

    auto modelIt = m_models.begin();
    while (modelIt != m_models.end()) {
        if (!modelIt->parentName.isEmpty()) {
            for (auto& mesh : modelIt->meshes) {
                if (mesh.vboInitialized) {
                    glDeleteBuffers(1, &mesh.vbo);
                    glDeleteBuffers(1, &mesh.ibo);
                }
            }
            modelIt = m_models.erase(modelIt);
        } else {
            ++modelIt;
        }
    }

    // Step 3: Sort grids by height
    std::vector<Grid2D*> sortedGrids;
    for (auto& grid : m_grids2D) {
        sortedGrids.push_back(&grid);
    }
    std::sort(sortedGrids.begin(), sortedGrids.end(),
              [](const Grid2D* a, const Grid2D* b) { return a->height < b->height; });

    // Step 4: Split objects based on invisible grid regions
    for (size_t i = 0; i < sortedGrids.size(); ++i) {
        if (!sortedGrids[i]->visible) {
            float lowerHeight = sortedGrids[i]->height;
            float upperHeight = std::numeric_limits<float>::max();
            for (size_t j = i + 1; j < sortedGrids.size(); ++j) {
                if (sortedGrids[j]->visible) {
                    upperHeight = sortedGrids[j]->height;
                    break;
                }
            }

            // Split point clouds
            for (const auto& pc : m_pointClouds) {
                if (!pc.parentName.isEmpty()) continue; // Skip already split objects
                splitPointCloudByHeight(pc.name, lowerHeight, upperHeight, true);
            }

            // Split models
            for (const auto& model : m_models) {
                if (!model.parentName.isEmpty()) continue; // Skip already split objects
                splitModelByHeight(model.name, lowerHeight, upperHeight, true);
            }
        }
    }

    m_sceneBoundsValid = false;
    update();
    doneCurrent();
}

void OpenGLWidget::splitModelByHeight(const QString& name, float lowerHeight, float upperHeight, bool hideMiddleSection)
{
    makeCurrent();
    auto it = std::find_if(m_models.begin(), m_models.end(),
                           [&name](const Model& model) { return model.name == name; });
    if (it == m_models.end()) return;

    Model original = *it;

    std::vector<Mesh> belowMeshes, betweenMeshes, aboveMeshes;
    for (const auto& mesh : original.meshes) {
        Mesh belowMesh, betweenMesh, aboveMesh;
        belowMesh.normals = betweenMesh.normals = aboveMesh.normals = mesh.normals;

        std::vector<size_t> belowVertexMap, betweenVertexMap, aboveVertexMap;
        for (size_t i = 0; i < mesh.vertices.size(); ++i) {
            float y = mesh.vertices[i].y();
            if (y < lowerHeight) {
                belowVertexMap.push_back(belowMesh.vertices.size());
                belowMesh.vertices.push_back(mesh.vertices[i]);
            } else if (y >= lowerHeight && y <= upperHeight) {
                betweenVertexMap.push_back(betweenMesh.vertices.size());
                betweenMesh.vertices.push_back(mesh.vertices[i]);
            } else {
                aboveVertexMap.push_back(aboveMesh.vertices.size());
                aboveMesh.vertices.push_back(mesh.vertices[i]);
            }
        }

        for (size_t i = 0; i < mesh.indices.size(); i += 3) {
            bool allBelow = true, allBetween = true, allAbove = true;
            for (int j = 0; j < 3; ++j) {
                size_t idx = mesh.indices[i + j];
                float y = mesh.vertices[idx].y();
                if (!(y < lowerHeight)) allBelow = false;
                if (y < lowerHeight || y > upperHeight) allBetween = false;
                if (!(y > upperHeight)) allAbove = false;
            }
            if (allBelow) {
                belowMesh.indices.push_back(belowVertexMap[mesh.indices[i]]);
                belowMesh.indices.push_back(belowVertexMap[mesh.indices[i + 1]]);
                belowMesh.indices.push_back(belowVertexMap[mesh.indices[i + 2]]);
            }
            if (allBetween) {
                betweenMesh.indices.push_back(betweenVertexMap[mesh.indices[i]]);
                betweenMesh.indices.push_back(betweenVertexMap[mesh.indices[i + 1]]);
                betweenMesh.indices.push_back(betweenVertexMap[mesh.indices[i + 2]]);
            }
            if (allAbove) {
                aboveMesh.indices.push_back(aboveVertexMap[mesh.indices[i]]);
                aboveMesh.indices.push_back(aboveVertexMap[mesh.indices[i + 1]]);
                aboveMesh.indices.push_back(aboveVertexMap[mesh.indices[i + 2]]);
            }
        }

        if (!belowMesh.vertices.empty()) {
            belowMesh.vertexCount = belowMesh.vertices.size();
            belowMesh.indexCount = belowMesh.indices.size();
            belowMesh.vboInitialized = false;
            belowMeshes.push_back(belowMesh);
        }
        if (!betweenMesh.vertices.empty()) {
            betweenMesh.vertexCount = betweenMesh.vertices.size();
            betweenMesh.indexCount = betweenMesh.indices.size();
            betweenMesh.vboInitialized = false;
            betweenMeshes.push_back(betweenMesh);
        }
        if (!aboveMesh.vertices.empty()) {
            aboveMesh.vertexCount = aboveMesh.vertices.size();
            aboveMesh.indexCount = aboveMesh.indices.size();
            aboveMesh.vboInitialized = false;
            aboveMeshes.push_back(aboveMesh);
        }
    }

    // Save original visibility state
    original.originalVisibility = original.visible;

    // Don't remove the original - just remember the splitting has occurred
    // We'll handle visibility later
    it->visible = false;

    if (!belowMeshes.empty()) {
        Model belowModel = original;
        belowModel.name = original.name + "_Below";
        belowModel.meshes = belowMeshes;
        belowModel.parentName = original.name;
        belowModel.visible = original.originalVisibility;
        calculateBoundingBox(belowMeshes.front().vertices, belowModel.min, belowModel.max);
        m_models.push_back(belowModel);
    }

    if (!betweenMeshes.empty()) {
        Model betweenModel = original;
        betweenModel.name = original.name + "_Between";
        betweenModel.meshes = betweenMeshes;
        betweenModel.parentName = original.name;
        betweenModel.visible = original.originalVisibility && !hideMiddleSection;
        calculateBoundingBox(betweenMeshes.front().vertices, betweenModel.min, betweenModel.max);
        m_models.push_back(betweenModel);
    }

    if (!aboveMeshes.empty()) {
        Model aboveModel = original;
        aboveModel.name = original.name + "_Above";
        aboveModel.meshes = aboveMeshes;
        aboveModel.parentName = original.name;
        aboveModel.visible = original.originalVisibility;
        calculateBoundingBox(aboveMeshes.front().vertices, aboveModel.min, aboveModel.max);
        m_models.push_back(aboveModel);
    }

    m_sceneBoundsValid = false;
    update();
    doneCurrent();
}

void OpenGLWidget::splitPointCloudByHeight(const QString& name, float lowerHeight, float upperHeight, bool hideMiddleSection)
{
    makeCurrent();
    auto it = std::find_if(m_pointClouds.begin(), m_pointClouds.end(),
                           [&name](const PointCloud& pc) { return pc.name == name; });
    if (it == m_pointClouds.end()) return;

    PointCloud original = *it;

    // Don't remove the original - just remember the splitting has occurred
    // We'll handle visibility later
    // m_pointClouds.erase(it);  <-- This line is removed

    std::vector<QVector3D> belowPoints, belowColors, betweenPoints, betweenColors, abovePoints, aboveColors;
    for (size_t i = 0; i < original.points.size(); ++i) {
        float y = original.points[i].y();
        if (y < lowerHeight) {
            belowPoints.push_back(original.points[i]);
            belowColors.push_back(original.colors[i]);
        } else if (y >= lowerHeight && y <= upperHeight) {
            betweenPoints.push_back(original.points[i]);
            betweenColors.push_back(original.colors[i]);
        } else {
            abovePoints.push_back(original.points[i]);
            aboveColors.push_back(original.colors[i]);
        }
    }

    // Save original visibility state
    original.originalVisibility = original.visible;

    // Hide original point cloud - we'll display split sections
    it->visible = false;

    // Create new point clouds for each section
    if (!belowPoints.empty()) {
        PointCloud belowPC = original;
        belowPC.name = original.name + "_Below";
        belowPC.points = belowPoints;
        belowPC.colors = belowColors;
        belowPC.pointCount = belowPoints.size();
        belowPC.parentName = original.name;
        belowPC.vboInitialized = false;
        belowPC.visible = original.originalVisibility; // Keep original visibility
        calculateBoundingBox(belowPoints, belowPC.min, belowPC.max);
        m_pointClouds.push_back(belowPC);
    }

    if (!betweenPoints.empty()) {
        PointCloud betweenPC = original;
        betweenPC.name = original.name + "_Between";
        betweenPC.points = betweenPoints;
        betweenPC.colors = betweenColors;
        betweenPC.pointCount = betweenPoints.size();
        betweenPC.parentName = original.name;
        betweenPC.vboInitialized = false;
        betweenPC.visible = original.originalVisibility && !hideMiddleSection; // Hide middle section if requested
        calculateBoundingBox(betweenPoints, betweenPC.min, betweenPC.max);
        m_pointClouds.push_back(betweenPC);
    }

    if (!abovePoints.empty()) {
        PointCloud abovePC = original;
        abovePC.name = original.name + "_Above";
        abovePC.points = abovePoints;
        abovePC.colors = aboveColors;
        abovePC.pointCount = abovePoints.size();
        abovePC.parentName = original.name;
        abovePC.vboInitialized = false;
        abovePC.visible = original.originalVisibility; // Keep original visibility
        calculateBoundingBox(abovePoints, abovePC.min, abovePC.max);
        m_pointClouds.push_back(abovePC);
    }

    m_sceneBoundsValid = false;
    update();
    doneCurrent();
}


void OpenGLWidget::convert2DTo3DGrid(const QString& name, int layers, float cellSize)
{
    for (const auto& grid2D : m_grids2D) {
        if (grid2D.name == name && grid2D.selected) {
            // Create a new 3D grid based on the 2D grid's properties
            QString newName = QString("Grid3D_%1x%2x%3").arg(grid2D.rows).arg(grid2D.columns).arg(layers);
            add3DGrid(grid2D.rows, grid2D.columns, layers, cellSize, newName);

            // Update selection state
            set2DGridSelected(name, false);
            set3DGridSelected(newName, true);

            m_sceneBoundsValid = false;
            qDebug() << "Converted 2D Grid '" << name << "' to 3D Grid '" << newName
                     << "' with" << grid2D.rows << "rows," << grid2D.columns << "columns, and" << layers << "layers";
            update();
            break;
        }
    }
}

void OpenGLWidget::updatePointCloud(const QString& name, const std::vector<QVector3D>& points, const std::vector<QVector3D>& colors, bool isMeshData)
{
    makeCurrent(); // Set OpenGL context

    for (auto& pc : m_pointClouds) {
        if (pc.name == name) {
            // Update points and colors
            pc.points = points;
            pc.colors = colors;
            pc.pointCount = static_cast<int>(points.size());
            pc.isMeshData = isMeshData; // Track if this is mesh data

            // Clean up existing VBO if initialized
            if (pc.vboInitialized && pc.vbo != 0) {
                glDeleteBuffers(1, &pc.vbo);
                pc.vbo = 0;
                pc.vboInitialized = false;
            }

            // Recalculate bounding box
            calculateBoundingBox(pc.points, pc.min, pc.max);

            // Reinitialize VBO with new data
            initializePointCloudVBO(pc);

            // Invalidate scene bounds to trigger recalculation
            m_sceneBoundsValid = false;

            // qDebug() << "Updated" << (pc.isMeshData ? "mesh-derived" : "point cloud") << "'"
            //          << name << "' with" << points.size() << "points";

            update(); // Request repaint
            break;
        }
    }

    doneCurrent(); // Release OpenGL context
}


void OpenGLWidget::processPointCloud(const QString& name, const QString& operation)
{
    for (const auto& pc : m_pointClouds) {
        if (pc.name == name) {
            // Create a sampling processor dialog
            SamplingProcessor processor(this);
            processor.setPointCloudData(pc.points, pc.colors, operation);

            // Connect signal from processor to handle processed point cloud
            connect(&processor, &SamplingProcessor::samplingApplied, this,
                    [this, name](const std::vector<QVector3D>& points,
                                 const std::vector<QVector3D>& colors,
                                 const QString& op) {
                        bool isMeshData = (op == "Sampling");
                        updatePointCloud(name, points, colors, isMeshData);
                    });

            // Show dialog
            processor.exec();
            break;
        }
    }
}

void OpenGLWidget::convertMeshToPointCloud(const QString& name)
{
    for (auto& model : m_models) {
        if (model.name == name) {
            SamplingProcessor processor(this);

            // Collect mesh data (vertices and indices)
            std::vector<QVector3D> vertices;
            std::vector<unsigned int> indices;
            for (const auto& mesh : model.meshes) {
                vertices.insert(vertices.end(), mesh.vertices.begin(), mesh.vertices.end());
                indices.insert(indices.end(), mesh.indices.begin(), mesh.indices.end());
            }
            std::vector<QVector3D> colors(vertices.size(), QVector3D(1.0f, 1.0f, 1.0f));

            processor.setPointCloudData(vertices, colors, "Sampling");
            processor.setMeshData(vertices, indices);

            connect(&processor, &SamplingProcessor::samplingApplied, this,
                    [this, name](const std::vector<QVector3D>& points,
                                 const std::vector<QVector3D>& colors,
                                 const QString& op) {
                        QString newName = name + "_PointCloud";
                        addPointCloud(points, colors, newName);

                        // Update point cloud properties
                        for (auto& pc : m_pointClouds) {
                            if (pc.name == newName) {
                                pc.isMeshData = true; // Mark as mesh-derived
                                calculateBoundingBox(points, pc.min, pc.max);
                                initializePointCloudVBO(pc);
                                break;
                            }
                        }

                        // Update selection state
                        setModelSelected(name, false);
                        setPointCloudSelected(newName, true);

                        // Update scene bounds and trigger redraw
                        updateSceneBounds();
                        update(); // Ensure the widget redraws to display the new point cloud

                        qDebug() << "Converted mesh '" << name << "' to point cloud '" << newName
                                 << "' with" << points.size() << "points";
                    });

            processor.exec();
            break;
        }
    }
}
void OpenGLWidget::sampleMeshSurface(const Model& model, std::vector<QVector3D>& points, std::vector<QVector3D>& colors, int sampleCount)
{
    if (model.meshes.empty() || sampleCount <= 0) return;

    // Collect triangles and compute areas
    struct Triangle {
        QVector3D v0, v1, v2;
        float area;
    };
    std::vector<Triangle> triangles;
    std::vector<float> areas;
    float totalArea = 0.0f;

    for (const auto& mesh : model.meshes) {
        for (size_t i = 0; i < mesh.indices.size(); i += 3) {
            if (i + 2 >= mesh.indices.size()) continue;

            unsigned int i0 = mesh.indices[i];
            unsigned int i1 = mesh.indices[i + 1];
            unsigned int i2 = mesh.indices[i + 2];

            if (i0 >= mesh.vertices.size() || i1 >= mesh.vertices.size() || i2 >= mesh.vertices.size()) continue;

            Triangle tri;
            tri.v0 = mesh.vertices[i0];
            tri.v1 = mesh.vertices[i1];
            tri.v2 = mesh.vertices[i2];

            // Compute triangle area using cross product
            QVector3D e1 = tri.v1 - tri.v0;
            QVector3D e2 = tri.v2 - tri.v0;
            tri.area = 0.5f * QVector3D::crossProduct(e1, e2).length();

            triangles.push_back(tri);
            areas.push_back(tri.area);
            totalArea += tri.area;
        }
    }

    if (triangles.empty() || totalArea == 0.0f) return;

    // Normalize areas for cumulative distribution
    std::vector<float> cumulativeAreas(areas.size());
    cumulativeAreas[0] = areas[0] / totalArea;
    for (size_t i = 1; i < areas.size(); ++i) {
        cumulativeAreas[i] = cumulativeAreas[i - 1] + (areas[i] / totalArea);
    }

    // Random number generation
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> dist(0.0f, 1.0f);

    points.reserve(sampleCount);
    colors.reserve(sampleCount);

    // Generate sample points
    for (int i = 0; i < sampleCount; ++i) {
        // Select triangle based on area
        float r = dist(gen);
        auto it = std::lower_bound(cumulativeAreas.begin(), cumulativeAreas.end(), r);
        size_t triIndex = std::distance(cumulativeAreas.begin(), it);
        if (triIndex >= triangles.size()) triIndex = triangles.size() - 1;

        const Triangle& tri = triangles[triIndex];

        // Generate random barycentric coordinates
        float r1 = dist(gen);
        float r2 = dist(gen);
        if (r1 + r2 > 1.0f) {
            r1 = 1.0f - r1;
            r2 = 1.0f - r2;
        }
        float r3 = 1.0f - r1 - r2;

        // Compute point position
        QVector3D point = r1 * tri.v0 + r2 * tri.v1 + r3 * tri.v2;
        points.push_back(point);

        // Assign default white color
        colors.push_back(QVector3D(1.0f, 1.0f, 1.0f));
    }
}

void OpenGLWidget::showEvent(QShowEvent *event)
{
    qDebug() << "OpenGLWidget::showEvent (Preview:" << m_isPreview << ", 2DMode:" << m_is2DMode << ")";
    setRenderingEnabled(true);
    QOpenGLWidget::showEvent(event);
}

void OpenGLWidget::hideEvent(QHideEvent *event)
{
    qDebug() << "OpenGLWidget::hideEvent (Preview:" << m_isPreview << ", 2DMode:" << m_is2DMode << ")";
    setRenderingEnabled(false);

    for (auto& pc : m_pointClouds) {
        if (pc.vboInitialized) {
            pc.vboInitialized = false;
            qDebug() << "Marked VBO as invalid for point cloud '" << pc.name << "' due to hide event";
        }
    }

    QOpenGLWidget::hideEvent(event);
}

void OpenGLWidget::resizeEvent(QResizeEvent *event)
{
    qDebug() << "OpenGLWidget::resizeEvent (Preview:" << m_isPreview << ", 2DMode:" << m_is2DMode << ") - Size:" << event->size();
    QOpenGLWidget::resizeEvent(event);
}

// void OpenGLWidget::clusterPointCloud(const QString& name, float voxelSize, float sampleFraction,
//                                      float planeDistanceThreshold, float clusterEps,
//                                      size_t maxPlaneIterations, size_t clusterMinPts)
// {
//     makeCurrent();

//     for (auto& pc : m_pointClouds) {
//         if (pc.name != name) continue;

//         // Convert point cloud to Vertex format
//         QVector<Vertex> combinedCloud;
//         combinedCloud.reserve(pc.points.size());
//         for (size_t i = 0; i < pc.points.size(); ++i) {
//             Vertex v;
//             v.position = pc.points[i];
//             combinedCloud.push_back(v);
//         }

//         // Perform clustering in a separate thread
//         std::thread clusteringThread([this, name, combinedCloud, voxelSize, sampleFraction,
//                                       planeDistanceThreshold, clusterEps, maxPlaneIterations,
//                                       clusterMinPts, &pc]() mutable {
//             try {
//                 std::vector<int> combinedLabels;
//                 Eigen::Vector4f planeModel;
//                 std::map<int, QVector3D> clusterColors;

//                 // Process the point cloud using the clustering function
//                 processPointCloudClustering(combinedCloud, combinedLabels, sampleFraction, planeDistanceThreshold,
//                                             clusterEps, maxPlaneIterations, clusterMinPts, planeModel);

//                 // Assign colors based on cluster labels
//                 std::vector<QVector3D> newColors(combinedCloud.size());
//                 std::random_device rd;
//                 std::mt19937 gen(rd());
//                 std::uniform_real_distribution<float> dist(0.0f, 1.0f);

//                 for (size_t i = 0; i < combinedLabels.size(); ++i) {
//                     int label = combinedLabels[i];
//                     if (label >= 0) { // Valid cluster or plane
//                         if (clusterColors.find(label) == clusterColors.end()) {
//                             clusterColors[label] = QVector3D(dist(gen), dist(gen), dist(gen));
//                         }
//                         newColors[i] = clusterColors[label];
//                     } else {
//                         newColors[i] = QVector3D(0.5f, 0.5f, 0.5f); // Gray for noise
//                     }
//                 }

//                 // Update point cloud with new points, colors, and labels
//                 std::vector<QVector3D> newPoints(combinedCloud.size());
//                 std::vector<int> newLabels(combinedLabels.size());
//                 for (size_t i = 0; i < combinedCloud.size(); ++i) {
//                     newPoints[i] = combinedCloud[i].position;
//                     newLabels[i] = combinedLabels[i];
//                 }

//                 // Update the point cloud in the main thread
//                 QMetaObject::invokeMethod(this, [this, name, newPoints, newColors, newLabels, isMeshData = pc.isMeshData, clusterColors]() {
//                     // Update points, colors, and labels
//                     for (auto& p : m_pointClouds) {
//                         if (p.name == name) {
//                             p.points = newPoints;
//                             p.colors = newColors;
//                             p.labels = newLabels;
//                             p.pointCount = newPoints.size();
//                             p.updateBoundingBox();
//                             updatePointCloudVBO(p);
//                             break;
//                         }
//                     }
//                     qDebug() << "Clustered point cloud '" << name << "' with" << newPoints.size()
//                              << "points and" << clusterColors.size() << "clusters";
//                     update();
//                 }, Qt::QueuedConnection);
//             } catch (const std::exception& e) {
//                 qDebug() << "Clustering thread exception:" << e.what();
//                 QMetaObject::invokeMethod(this, "update", Qt::QueuedConnection);
//             }
//         });

//         clusteringThread.detach(); // Detach the thread to run independently
//         break;
//     }

//     doneCurrent();
// }

void OpenGLWidget::clusterPointCloud(const QString& name, float voxelSize, float sampleFraction,
                                     float planeDistanceThreshold, float clusterEps,
                                     size_t maxPlaneIterations, size_t clusterMinPts,
                                     float minPlaneInlierRatio) {
    makeCurrent();

    for (auto& pc : m_pointClouds) {
        if (pc.name != name) continue;

        // Convert point cloud to Vertex format
        QVector<Vertex> combinedCloud;
        combinedCloud.reserve(pc.points.size());
        for (size_t i = 0; i < pc.points.size(); ++i) {
            Vertex v;
            v.position = pc.points[i];
            combinedCloud.push_back(v);
        }

        // Create progress dialog
        ProgressDialog* progressDialog = new ProgressDialog(this);
        progressDialog->show();
        QApplication::processEvents(); // Ensure the dialog is displayed

        // Perform clustering in a separate thread
        std::thread clusteringThread([this, name, combinedCloud, voxelSize, sampleFraction,
                                      planeDistanceThreshold, clusterEps, maxPlaneIterations,
                                      clusterMinPts, minPlaneInlierRatio, progressDialog]() mutable {
            try {
                std::vector<int> combinedLabels;
                Eigen::Vector4f planeModel;
                std::map<int, QVector3D> clusterColors;
                std::vector<ClusterInfo> clusterInfos;
                std::atomic<int> progress{0};

                // Process the point cloud
                processPointCloudClustering(combinedCloud, combinedLabels, voxelSize, sampleFraction,
                                            planeDistanceThreshold, clusterEps, maxPlaneIterations,
                                            clusterMinPts, minPlaneInlierRatio, planeModel, clusterInfos, progress);

                // Update progress dialog during clustering
                int lastProgress = -1;
                while (progress < 100) {
                    int currentProgress = progress.load(std::memory_order_relaxed);
                    if (currentProgress != lastProgress) {
                        QMetaObject::invokeMethod(progressDialog,
                                                  [progressDialog, p = currentProgress]() {
                                                      progressDialog->updateProgress(p); // Use updateProgress as defined in ProgressDialog
                                                  },
                                                  Qt::QueuedConnection);
                        lastProgress = currentProgress;
                    }
                    // Process events in the main thread to ensure UI responsiveness
                    QMetaObject::invokeMethod(qApp, []() {
                        QApplication::processEvents();
                    }, Qt::QueuedConnection);
                    std::this_thread::sleep_for(std::chrono::milliseconds(10)); // Further reduced interval for smoother updates
                }

                // Ensure final progress update
                QMetaObject::invokeMethod(progressDialog,
                                          [progressDialog]() {
                                              progressDialog->updateProgress(100); // Use updateProgress for final update
                                          },
                                          Qt::QueuedConnection);
                QMetaObject::invokeMethod(qApp, []() {
                    QApplication::processEvents();
                }, Qt::QueuedConnection);

                // Assign colors based on cluster labels
                std::vector<QVector3D> newColors(combinedCloud.size());
                std::random_device rd;
                std::mt19937 gen(rd());
                std::uniform_real_distribution<float> dist(0.0f, 1.0f);

                for (size_t i = 0; i < combinedLabels.size(); ++i) {
                    int label = combinedLabels[i];
                    if (label >= 0) {
                        if (clusterColors.find(label) == clusterColors.end()) {
                            clusterColors[label] = QVector3D(dist(gen), dist(gen), dist(gen));
                        }
                        newColors[i] = clusterColors[label];
                    } else {
                        newColors[i] = QVector3D(0.5f, 0.5f, 0.5f); // Gray for noise
                    }
                }

                // Update point cloud with new points, colors, and labels
                std::vector<QVector3D> newPoints(combinedCloud.size());
                std::vector<int> newLabels(combinedLabels.size());
                for (size_t i = 0; i < combinedCloud.size(); ++i) {
                    newPoints[i] = combinedCloud[i].position;
                    newLabels[i] = combinedLabels[i];
                }

                // Update the point cloud and show dialog in the main thread
                QMetaObject::invokeMethod(this, [this, name, newPoints, newColors, newLabels, clusterColors, clusterInfos, progressDialog]() {
                    for (auto& p : m_pointClouds) {
                        if (p.name == name) {
                            p.points = newPoints;
                            p.colors = newColors;
                            p.labels = newLabels;
                            p.pointCount = newPoints.size();
                            p.updateBoundingBox();
                            updatePointCloudVBO(p);
                            break;
                        }
                    }
                    qDebug() << "Clustered point cloud '" << name << "' with" << newPoints.size()
                             << "points and" << clusterColors.size() << "clusters";

                    // Show results dialog
                    ClusterResultsDialog dialog(clusterInfos, this);
                    dialog.exec();

                    // Clean up progress dialog
                    progressDialog->close(); // Close to ensure proper cleanup
                    delete progressDialog;
                    update();
                }, Qt::QueuedConnection);
            } catch (const std::exception& e) {
                qDebug() << "Clustering thread exception:" << e.what();
                QMetaObject::invokeMethod(this, [progressDialog]() {
                    progressDialog->close();
                    delete progressDialog;
                }, Qt::QueuedConnection);
                QMetaObject::invokeMethod(this, "update", Qt::QueuedConnection);
            }
        });

        clusteringThread.detach();
        break;
    }

    doneCurrent();
}
void OpenGLWidget::identifySimilarClusters(const QString& name)
{
    makeCurrent();

    // Find the selected point cloud
    PointCloud* targetCloud = nullptr;
    for (auto& pc : m_pointClouds) {
        if (pc.name == name) {
            targetCloud = &pc;
            break;
        }
    }

    if (!targetCloud || targetCloud->labels.empty()) {
        qDebug() << "Point cloud not found or not clustered:" << name;
        doneCurrent();
        return;
    }

    // Group points by cluster label
    std::map<int, std::vector<size_t>> clusterIndices;
    for (size_t i = 0; i < targetCloud->labels.size(); ++i) {
        if (targetCloud->labels[i] >= 0) { // Valid cluster (exclude noise)
            clusterIndices[targetCloud->labels[i]].push_back(i);
        }
    }

    if (clusterIndices.empty()) {
        qDebug() << "No valid clusters found for similarity identification.";
        doneCurrent();
        return;
    }

    // Compute bounding boxes and point counts for each cluster
    struct BoundingBox {
        QVector3D center;
        QVector3D size;
    };

    std::unordered_map<int, BoundingBox> clusterBoxes;
    std::unordered_map<int, int> clusterPointCounts;
    std::vector<int> uniqueClusters;

    // Find unique clusters and compute bounding boxes
    std::unordered_set<int> uniqueLabelsSet;
    for (size_t i = 0; i < targetCloud->labels.size(); ++i) {
        int label = targetCloud->labels[i];
        if (label >= 0) {
            uniqueLabelsSet.insert(label);
            clusterPointCounts[label]++;
        }
    }

    uniqueClusters.reserve(uniqueLabelsSet.size());
    for (int label : uniqueLabelsSet) {
        BoundingBox bbox;
        QVector3D minPoint(std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
        QVector3D maxPoint(std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest());
        size_t pointCount = 0;

        for (size_t i = 0; i < targetCloud->labels.size(); ++i) {
            if (targetCloud->labels[i] == label) {
                const QVector3D& pos = targetCloud->points[i];
                minPoint.setX(std::min(minPoint.x(), pos.x()));
                minPoint.setY(std::min(minPoint.y(), pos.y()));
                minPoint.setZ(std::min(minPoint.z(), pos.z()));
                maxPoint.setX(std::max(maxPoint.x(), pos.x()));
                maxPoint.setY(std::max(maxPoint.y(), pos.y()));
                maxPoint.setZ(std::max(maxPoint.z(), pos.z()));
                pointCount++;
            }
        }

        if (pointCount > 0) {
            bbox.size = maxPoint - minPoint;
            bbox.center = (maxPoint + minPoint) * 0.5f;
            if (bbox.size.length() > 0) {
                clusterBoxes[label] = bbox;
                uniqueClusters.push_back(label);
            }
        }
    }

    if (uniqueClusters.empty()) {
        qDebug() << "No valid clusters with non-zero bounding boxes found.";
        doneCurrent();
        return;
    }

    // Similarity parameters (fixed as in similarclusters.cpp)
    const float dimTolerance = 0.1f;     // 10% tolerance for dimensions
    const float minDimension = 0.01f;    // Minimum dimension to avoid division by zero
    const float pointCountTolerance = 0.2f;  // 20% tolerance for point count

    // Group similar clusters
    std::unordered_map<int, int> clusterToGroup;
    std::vector<bool> processed(uniqueClusters.size(), false);
    int groupId = 0;

    for (size_t i = 0; i < uniqueClusters.size(); ++i) {
        if (processed[i]) continue;

        int clusterI = uniqueClusters[i];
        clusterToGroup[clusterI] = groupId;
        processed[i] = true;

        BoundingBox boxI = clusterBoxes[clusterI];
        QVector3D sizeI(
            std::max(boxI.size.x(), minDimension),
            std::max(boxI.size.y(), minDimension),
            std::max(boxI.size.z(), minDimension)
            );
        int pointCountI = clusterPointCounts[clusterI];

        for (size_t j = 0; j < uniqueClusters.size(); ++j) {
            if (i == j || processed[j]) continue;

            int clusterJ = uniqueClusters[j];
            BoundingBox boxJ = clusterBoxes[clusterJ];
            QVector3D sizeJ(
                std::max(boxJ.size.x(), minDimension),
                std::max(boxJ.size.y(), minDimension),
                std::max(boxJ.size.z(), minDimension)
                );
            int pointCountJ = clusterPointCounts[clusterJ];

            // Check dimension similarity
            QVector3D dimDiffRatio(
                std::abs(sizeI.x() - sizeJ.x()) / sizeI.x(),
                std::abs(sizeI.y() - sizeJ.y()) / sizeI.y(),
                std::abs(sizeI.z() - sizeJ.z()) / sizeI.z()
                );
            bool dimsSimilar = (
                dimDiffRatio.x() <= dimTolerance &&
                dimDiffRatio.y() <= dimTolerance &&
                dimDiffRatio.z() <= dimTolerance
                );

            // Check point count similarity
            float pointCountDiff = std::abs(pointCountI - pointCountJ) / static_cast<float>(std::max(1, pointCountI));
            bool pointCountSimilar = (pointCountDiff <= pointCountTolerance);

            if (dimsSimilar && pointCountSimilar) {
                clusterToGroup[clusterJ] = groupId;
                processed[j] = true;
            }
        }
        groupId++;
    }

    // Assign colors to points based on group IDs
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> dis(0.0f, 1.0f);
    std::map<int, QVector3D> groupColors;

    for (int group = 0; group < groupId; ++group) {
        groupColors[group] = QVector3D(dis(gen), dis(gen), dis(gen));
    }

    for (size_t i = 0; i < targetCloud->labels.size(); ++i) {
        int label = targetCloud->labels[i];
        if (label >= 0 && clusterToGroup.find(label) != clusterToGroup.end()) {
            int group = clusterToGroup[label];
            targetCloud->colors[i] = groupColors[group];
        } else {
            targetCloud->colors[i] = QVector3D(0.5f, 0.5f, 0.5f); // Gray for noise or ungrouped
        }
    }

    // Update VBO with new colors
    updatePointCloudVBO(*targetCloud);

    // Log results
    qDebug() << "Identified" << groupId << "groups of similar clusters.";
    doneCurrent();
}

void OpenGLWidget::captureViewport(const QString& name)
{
    // Create a new viewport capture with minimal data
    ViewportCapture capture;
    capture.name = name;
    capture.rotation = QVector3D(m_xRotation, m_yRotation, 0.0f);
    capture.panOffset = m_panOffset;
    capture.zoom = m_zoom;

    // Store it in the captures list
    m_viewportCaptures.push_back(capture);

    qDebug() << "Viewport captured:" << name;
}

void OpenGLWidget::addViewportCapture(const ViewportCapture& capture)
{
    // Check if a capture with this name already exists
    for (int i = 0; i < m_viewportCaptures.size(); ++i) {
        if (m_viewportCaptures[i].name == capture.name) {
            // Update existing capture
            m_viewportCaptures[i] = capture;
            return;
        }
    }

    // Add new capture
    m_viewportCaptures.append(capture);
}

QList<OpenGLWidget::ViewportCapture> OpenGLWidget::getViewportCaptures() const
{
    return m_viewportCaptures;
}

void OpenGLWidget::applyViewportCapture(const QString& name)
{
    // Find the capture with matching name
    for (const auto& capture : m_viewportCaptures) {
        if (capture.name == name) {
            // Apply the view parameters
            m_xRotation = capture.rotation.x();
            m_yRotation = capture.rotation.y();
            m_panOffset = capture.panOffset;
            m_zoom = capture.zoom;

            // Signal for redraw
            m_viewChanged = true;
            update();

            qDebug() << "Applied viewport:" << name;
            return;
        }
    }

    qDebug() << "Viewport not found:" << name;
}


// Add to OpenGLWidget class in openglwidget.cpp
QMatrix4x4 OpenGLWidget::getProjectionMatrix() const
{
    // If we haven't stored a projection matrix yet, calculate one based on current parameters
    if (m_projectionMatrix.isIdentity()) {
        QMatrix4x4 matrix;
        float aspectRatio = static_cast<float>(width()) / height();
        matrix.perspective(45.0f, aspectRatio, 0.1f, 1000.0f);
        return matrix;
    }
    return m_projectionMatrix;
}

void OpenGLWidget::setProjectionMatrix(const QMatrix4x4& matrix)
{
    m_projectionMatrix = matrix;
    m_viewChanged = true;
}

void OpenGLWidget::getCameraParameters(QVector3D& position, QVector3D& target, QVector3D& up) const
{
    // Calculate camera position based on current view parameters
    if (m_sceneBoundsValid) {
        float cameraDistance = m_sceneRadius * 2.0f / m_zoom;

        // Apply rotation to get actual camera position
        QMatrix4x4 rotMatrix;
        rotMatrix.rotate(m_xRotation, 1.0f, 0.0f, 0.0f);
        rotMatrix.rotate(m_yRotation, 0.0f, 1.0f, 0.0f);

        QVector3D cameraDir = rotMatrix.map(QVector3D(0.0f, 0.0f, 1.0f));
        position = m_sceneCenter + cameraDir * cameraDistance;
        position += m_panOffset;

        target = m_sceneCenter + m_panOffset;

        // Calculate up vector based on current rotation
        up = rotMatrix.map(QVector3D(0.0f, 1.0f, 0.0f));
    } else {
        // Default values if scene bounds are not valid
        position = QVector3D(0.0f, 0.0f, 5.0f) + m_panOffset;
        target = m_panOffset;
        up = QVector3D(0.0f, 1.0f, 0.0f);
    }
}

void OpenGLWidget::setCameraParameters(const QVector3D& position, const QVector3D& target, const QVector3D& up)
{
    // Store the target as scene center
    m_sceneCenter = target - m_panOffset;

    // Calculate distance from position to target
    float distance = (position - target).length();

    // Calculate zoom factor based on distance
    if (m_sceneBoundsValid) {
        m_zoom = m_sceneRadius * 2.0f / distance;
    } else {
        m_zoom = 5.0f / distance;
    }

    // Calculate rotation angles from the camera direction and up vector
    QVector3D dir = (target - position).normalized();

    // Calculate pitch (x-rotation)
    m_xRotation = qRadiansToDegrees(qAsin(-dir.y()));

    // Calculate yaw (y-rotation)
    m_yRotation = qRadiansToDegrees(qAtan2(-dir.x(), -dir.z()));

    m_viewChanged = true;
}
void OpenGLWidget::applySingleColor(const QString& name, const QVector3D& color)
{
    for (auto& pc : m_pointClouds) {
        if (pc.name == name) {
            pc.colors.clear();
            pc.colors.resize(pc.points.size(), color);
            updatePointCloudVBO(pc);
            update();
            break;
        }
    }
}

void OpenGLWidget::applyGradientColor(const QString& name, const QVector3D& startColor, const QVector3D& endColor)
{
    for (auto& pc : m_pointClouds) {
        if (pc.name == name) {
            if (pc.points.empty()) return;

            // Find the min and max Y values for gradient
            float minY = pc.points[0].y();
            float maxY = minY;
            for (const auto& point : pc.points) {
                minY = std::min(minY, point.y());
                maxY = std::max(maxY, point.y());
            }

            float range = maxY - minY;
            if (range == 0.0f) range = 1.0f; // Avoid division by zero

            pc.colors.clear();
            pc.colors.reserve(pc.points.size());
            for (const auto& point : pc.points) {
                float t = (point.y() - minY) / range;
                QVector3D color = startColor + (endColor - startColor) * t;
                pc.colors.push_back(color);
            }

            updatePointCloudVBO(pc);
            update();
            break;
        }
    }
}


bool OpenGLWidget::deleteObject(int objectId)
{
    // First check if any item is selected, regardless of ID

    // Check if any point cloud is selected
    for (size_t i = 0; i < m_pointClouds.size(); i++) {
        if (m_pointClouds[i].selected) {
            // Delete point cloud's OpenGL resources
            if (m_pointClouds[i].vboInitialized) {
                makeCurrent();
                glDeleteBuffers(1, &m_pointClouds[i].vbo);
                doneCurrent();
            }

            // Remove the point cloud
            m_pointClouds.erase(m_pointClouds.begin() + i);

            // Update scene bounds
            m_sceneBoundsValid = false;
            update();

            return true;
        }
    }

    // Check if any model or model node is selected
    for (size_t i = 0; i < m_models.size(); i++) {
        if (m_models[i].selected) {
            // Delete all OpenGL resources for the entire model
            makeCurrent();
            for (auto& node : m_models[i].nodes) {
                for (auto& mesh : node.meshes) {
                    if (mesh.vboInitialized) {
                        glDeleteBuffers(1, &mesh.vbo);
                        glDeleteBuffers(1, &mesh.ibo);
                    }
                }
            }
            doneCurrent();

            // Remove the model
            m_models.erase(m_models.begin() + i);

            // Update scene bounds
            m_sceneBoundsValid = false;
            update();

            return true;
        }

        // Check for selected nodes within the model
        bool nodeDeleted = false;
        for (size_t j = 0; j < m_models[i].nodes.size(); j++) {
            if (m_models[i].nodes[j].selected) {
                // Delete node's OpenGL resources
                makeCurrent();
                for (auto& mesh : m_models[i].nodes[j].meshes) {
                    if (mesh.vboInitialized) {
                        glDeleteBuffers(1, &mesh.vbo);
                        glDeleteBuffers(1, &mesh.ibo);
                    }
                }
                doneCurrent();

                // Remove this node from its parent's children list
                if (m_models[i].nodes[j].parentIndex >= 0) {
                    auto& parentNode = m_models[i].nodes[m_models[i].nodes[j].parentIndex];
                    auto it = std::find(parentNode.children.begin(), parentNode.children.end(), j);
                    if (it != parentNode.children.end()) {
                        parentNode.children.erase(it);
                    }
                }

                // Remove the node and update indices for all remaining nodes
                m_models[i].nodes.erase(m_models[i].nodes.begin() + j);

                // We need to update all child indices in remaining nodes
                for (auto& node : m_models[i].nodes) {
                    // Update parent index
                    if (node.parentIndex > static_cast<int>(j)) {
                        node.parentIndex--;
                    }

                    // Update children indices
                    for (auto& childIdx : node.children) {
                        if (childIdx > static_cast<int>(j)) {
                            childIdx--;
                        }
                    }
                }

                nodeDeleted = true;
                break;
            }
        }

        if (nodeDeleted) {
            // Update scene bounds
            m_sceneBoundsValid = false;
            update();
            return true;
        }
    }

    // Check if any 2D grid is selected
    for (size_t i = 0; i < m_grids2D.size(); i++) {
        if (m_grids2D[i].selected) {
            // Delete grid's OpenGL resources
            if (m_grids2D[i].vboInitialized) {
                makeCurrent();
                glDeleteBuffers(1, &m_grids2D[i].vbo);
                glDeleteBuffers(1, &m_grids2D[i].ibo);
                doneCurrent();
            }

            // Remove the grid
            m_grids2D.erase(m_grids2D.begin() + i);

            // Update scene bounds
            m_sceneBoundsValid = false;
            update();

            return true;
        }
    }

    // Check if any 3D grid is selected
    for (size_t i = 0; i < m_grids3D.size(); i++) {
        if (m_grids3D[i].selected) {
            // Delete grid's OpenGL resources
            if (m_grids3D[i].vboInitialized) {
                makeCurrent();
                glDeleteBuffers(1, &m_grids3D[i].vbo);
                glDeleteBuffers(1, &m_grids3D[i].ibo);
                doneCurrent();
            }

            // Remove the grid
            m_grids3D.erase(m_grids3D.begin() + i);

            // Update scene bounds
            m_sceneBoundsValid = false;
            update();

            return true;
        }
    }

    // If nothing was selected, try to use objectId as a fallback
    if (objectId >= 0) {
        // Check if objectId refers to a point cloud
        if (objectId < static_cast<int>(m_pointClouds.size())) {
            // Delete point cloud OpenGL resources
            if (m_pointClouds[objectId].vboInitialized) {
                makeCurrent();
                glDeleteBuffers(1, &m_pointClouds[objectId].vbo);
                doneCurrent();
            }

            // Remove the point cloud
            m_pointClouds.erase(m_pointClouds.begin() + objectId);

            // Update scene bounds
            m_sceneBoundsValid = false;
            update();

            return true;
        }

        // Check if objectId refers to a model
        objectId -= m_pointClouds.size();
        if (objectId >= 0 && objectId < static_cast<int>(m_models.size())) {
            // Delete model OpenGL resources
            makeCurrent();
            for (auto& node : m_models[objectId].nodes) {
                for (auto& mesh : node.meshes) {
                    if (mesh.vboInitialized) {
                        glDeleteBuffers(1, &mesh.vbo);
                        glDeleteBuffers(1, &mesh.ibo);
                    }
                }
            }
            doneCurrent();

            // Remove the model
            m_models.erase(m_models.begin() + objectId);

            // Update scene bounds
            m_sceneBoundsValid = false;
            update();

            return true;
        }
    }

    return false;
}


//viewport
void OpenGLWidget::setOrthographicView()
{
    // Set orthographic projection parameters
    m_isPerspective = false;

    // Position camera at an isometric-like angle for better 3D comprehension
    // while maintaining orthographic projection
    m_cameraPosition = QVector3D(3.0f, 3.0f, 5.0f);
    m_cameraTarget = QVector3D(0.0f, 0.0f, 0.0f);
    m_cameraUp = QVector3D(0.0f, 1.0f, 0.0f);

    // Set rotation angles to create a 3/4 view
    m_xRotation = 30.0f;  // Rotate down slightly
    m_yRotation = -45.0f; // Rotate to see multiple sides

    // Update projection matrix
    updateProjectionMatrix();
    // Mark the view as changed to trigger redraw
    m_viewChanged = true;
    // Trigger a redraw
    update();
    // Emit the viewportChanged signal
    emit viewportChanged();
}
void OpenGLWidget::updateProjectionMatrix()
{
    float aspectRatio = static_cast<float>(width()) / height();

    if (m_isPerspective) {
        // Perspective projection
        m_projectionMatrix.setToIdentity();
        m_projectionMatrix.perspective(45.0f, aspectRatio, 0.1f, 1000.0f);
    } else {
        // Orthographic projection
        float zoom = 10.0f / m_zoom;
        m_projectionMatrix.setToIdentity();
        m_projectionMatrix.ortho(-zoom * aspectRatio, zoom * aspectRatio,
                                 -zoom, zoom,
                                 0.1f, 1000.0f);
    }
}

void OpenGLWidget::resetCamera()
{
    updateSceneBounds();  // Ensure we have a valid bounding volume

    if (m_sceneBoundsValid && m_sceneRadius > 0.0f) {
        m_zoom = 2.0f / m_sceneRadius;
    } else {
        m_zoom = 1.0f;
    }

    m_panOffset = QVector3D(0, 0, 0);
    m_xRotation = 0.0f;
    m_yRotation = 0.0f;

    m_viewChanged = true;
    update();
}

void OpenGLWidget::setViewChanged(bool changed)
{
    m_viewChanged = changed;
}

void OpenGLWidget::setBottomView()
{
    m_xRotation = -90.0f; // Rotate to look up from below
    m_yRotation = 0.0f;
    m_viewChanged = true;
    update();
}
void OpenGLWidget::invalidateBuffers() {
    for (auto &pc : m_pointClouds) {
        pc.vboInitialized = false;
    }
    for (auto &model : m_models) {
        for (auto &node : model.nodes) {
            for (auto &mesh : node.meshes) {
                mesh.vboInitialized = false;
            }
        }
    }
    for (auto &g2d : m_grids2D) g2d.vboInitialized = false;
    for (auto &g3d : m_grids3D) g3d.vboInitialized = false;
}

void OpenGLWidget::addLightweightModel(const QString& name,
                                       const std::vector<QVector3D>& vertices,
                                       const std::vector<QVector3D>& normals,
                                       const std::vector<unsigned int>& indices)
{
    Model model;
    model.name = name;
    model.visible = true;

    Mesh mesh;
    mesh.name = "PreviewMesh";
    mesh.vertices = vertices;
    mesh.normals = normals;
    mesh.indices = indices;

    mesh.vboInitialized = false;  // will be initialized on draw
    model.meshes.push_back(mesh);

    // Calculate bounds
    QVector3D min(1e9,1e9,1e9), max(-1e9,-1e9,-1e9);
    for (const auto& v : vertices) {
        min.setX(qMin(min.x(), v.x()));
        min.setY(qMin(min.y(), v.y()));
        min.setZ(qMin(min.z(), v.z()));
        max.setX(qMax(max.x(), v.x()));
        max.setY(qMax(max.y(), v.y()));
        max.setZ(qMax(max.z(), v.z()));
    }
    model.min = min;
    model.max = max;

    m_models.push_back(model);
    m_sceneBoundsValid = false;
}


// --- Enhanced Rotation Center Control with Reliable Point Selection ---
// --- Final Responsive and Visual Rotation Point Selection Flow for .pts/.ply using VBOs ---

void OpenGLWidget::contextMenuEvent(QContextMenuEvent *event)
{
    if (m_isPreview) return;

    QMenu menu(this);
    QAction* setCenterAction = menu.addAction("Set Rotation Center (Alt + Click)");
    QAction* resetCenterAction = menu.addAction("Reset Rotation Center");

    QAction* selected = menu.exec(event->globalPos());

    if (selected == setCenterAction) {
        m_selectingRotationCenter = true;
        m_hasHoveredPoint = false;
        setCursor(Qt::CrossCursor);
        update();
    } else if (selected == resetCenterAction) {
        resetRotationCenter();
    }
}

void OpenGLWidget::resetRotationCenter()
{
    m_rotationCenterSet = false;
    m_selectingRotationCenter = false;
    m_hasHoveredPoint = false;
    setCursor(Qt::ArrowCursor);
    update();
}

// --- Revised Rotation Center Selection: Projected Fallback Support ---
// This version ensures a point is always chosen: either nearest existing or cursor-defined projection.
QVector3D OpenGLWidget::getCursorProjectedPoint(const QPoint& screenPos, bool& success)
{
    makeCurrent();

    GLint viewport[4];
    GLdouble modelview[16];
    GLdouble projection[16];
    glGetIntegerv(GL_VIEWPORT, viewport);
    glGetDoublev(GL_MODELVIEW_MATRIX, modelview);
    glGetDoublev(GL_PROJECTION_MATRIX, projection);

    float winX = screenPos.x();
    float winY = viewport[3] - screenPos.y(); // Flip Y for OpenGL
    float winZ = 1.0f;

    glReadPixels(winX, winY, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &winZ);

    GLdouble posX, posY, posZ;
    if (gluUnProject(winX, winY, winZ, modelview, projection, viewport, &posX, &posY, &posZ)) {
        success = true;
        return QVector3D(posX, posY, posZ);
    }

    success = false;
    return QVector3D();
}

QVector3D OpenGLWidget::findClosestPointOrProjected(const QPoint& screenPos, bool& usedProjection)
{
    const float MAX_SCREEN_DISTANCE = 20.0f;
    QVector3D closestPoint;
    float minDistance = MAX_SCREEN_DISTANCE;
    bool found = false;
    usedProjection = false;

    GLint viewport[4];
    GLdouble modelview[16];
    GLdouble projection[16];
    glGetIntegerv(GL_VIEWPORT, viewport);
    glGetDoublev(GL_MODELVIEW_MATRIX, modelview);
    glGetDoublev(GL_PROJECTION_MATRIX, projection);

    for (const auto& pc : m_pointClouds) {
        if (!pc.visible || pc.points.empty()) continue;

        QMatrix4x4 transform;
        transform.translate(pc.offset);
        transform.rotate(pc.rotation.x(), 1, 0, 0);
        transform.rotate(pc.rotation.y(), 0, 1, 0);
        transform.rotate(pc.rotation.z(), 0, 0, 1);
        transform.scale(pc.scale);

        for (const auto& pt : pc.points) {
            QVector3D world = transform.map(pt);
            GLdouble sx, sy, sz;
            if (gluProject(world.x(), world.y(), world.z(),
                           modelview, projection, viewport,
                           &sx, &sy, &sz)) {
                float dx = screenPos.x() - float(sx);
                float dy = (viewport[3] - screenPos.y()) - float(sy);
                float dist = std::sqrt(dx * dx + dy * dy);

                if (dist < minDistance) {
                    minDistance = dist;
                    closestPoint = world;
                    found = true;
                }
            }
        }
    }

    if (found) return closestPoint;

    // No existing point found, project mouse to 3D space (on scene center depth)
    usedProjection = true;
    GLdouble objX, objY, objZ;
    GLdouble winX = screenPos.x();
    GLdouble winY = viewport[3] - screenPos.y();

    // Use scene center depth as approximate fallback
    GLdouble sx, sy, sz;
    if (gluProject(m_sceneCenter.x(), m_sceneCenter.y(), m_sceneCenter.z(),
                   modelview, projection, viewport, &sx, &sy, &sz)) {
        if (gluUnProject(winX, winY, sz,
                         modelview, projection, viewport,
                         &objX, &objY, &objZ)) {
            return QVector3D(objX, objY, objZ);
        }
    }

    usedProjection = false;
    return QVector3D();
}


QVector3D OpenGLWidget::findClosestPointOnGeometry(const QPoint& screenPos, bool& pointFound)
{
    const float MAX_SCREEN_DISTANCE = 25.0f;
    const float MAX_SCREEN_DISTANCE_SQ = MAX_SCREEN_DISTANCE * MAX_SCREEN_DISTANCE;

    QVector3D closestPoint;
    float minDistanceSq = MAX_SCREEN_DISTANCE_SQ;
    pointFound = false;

    makeCurrent();

    GLint viewport[4];
    GLdouble modelview[16];
    GLdouble projection[16];
    glGetIntegerv(GL_VIEWPORT, viewport);
    glGetDoublev(GL_MODELVIEW_MATRIX, modelview);
    glGetDoublev(GL_PROJECTION_MATRIX, projection);

    for (const auto& pc : m_pointClouds) {
        if (!pc.visible || pc.points.empty()) continue;

        QMatrix4x4 transform;
        transform.translate(pc.offset);
        transform.rotate(pc.rotation.x(), 1, 0, 0);
        transform.rotate(pc.rotation.y(), 0, 1, 0);
        transform.rotate(pc.rotation.z(), 0, 0, 1);
        transform.scale(pc.scale);

        for (const auto& pt : pc.points) {
            QVector3D world = transform.map(pt);

            GLdouble sx, sy, sz;
            if (!gluProject(world.x(), world.y(), world.z(),
                            modelview, projection, viewport,
                            &sx, &sy, &sz)) continue;

            float dx = screenPos.x() - float(sx);
            float dy = (viewport[3] - screenPos.y()) - float(sy);
            float distSq = dx * dx + dy * dy;

            if (distSq < minDistanceSq) {
                minDistanceSq = distSq;
                closestPoint = world;
                pointFound = true;
            }
        }
    }

    return closestPoint;
}

// --- Optimized mouseMoveEvent for Responsive Hovering ---
void OpenGLWidget::mouseMoveEvent(QMouseEvent *event)
{
    if (m_isPreview) return;

    QPointF currentPos = event->position();

    if (m_selectingRotationCenter && (event->modifiers() & Qt::AltModifier)) {
        m_mouseScreenPos = event->pos();

        bool found = false;
        QVector3D hovered = findClosestPointOnGeometry(event->pos(), found);
        if (!found) hovered = findClosestFBXVertex(event->pos(), found);

        m_hasHoveredPoint = found;
        if (found) m_hoveredPoint = hovered;

        update();
    } else {
        int dx = static_cast<int>(currentPos.x() - m_lastPos.x());
        int dy = static_cast<int>(currentPos.y() - m_lastPos.y());

        if (m_rotating) {
            m_yRotation += dx * 0.5f;
            m_xRotation += dy * 0.5f;
            m_xRotation = std::clamp(m_xRotation, -89.0f, 89.0f);
            while (m_yRotation > 360.0f) m_yRotation -= 360.0f;
            while (m_yRotation < 0.0f) m_yRotation += 360.0f;
            m_viewChanged = true;
            update();
        } else if (m_panning) {
            float panSpeed = 0.01f * m_zoom;
            m_panOffset.setX(m_panOffset.x() + dx * panSpeed);
            m_panOffset.setY(m_panOffset.y() - dy * panSpeed);
            m_viewChanged = true;
            update();
        }
    }

    m_lastPos = currentPos;
}

void OpenGLWidget::mousePressEvent(QMouseEvent *event)
{
    if (m_isPreview) return;

    m_lastPos = event->position();

    if (event->button() == Qt::LeftButton) {
        if (m_selectingRotationCenter && (event->modifiers() & Qt::AltModifier)) {
            bool found = false;
            QVector3D selected = findClosestPointOnGeometry(event->pos(), found);

            if (!found) selected = findClosestFBXVertex(event->pos(), found);
            if (!found) selected = findFallbackFBXPickPoint(event->pos(), found);

            if (found) {
                m_rotationCenter = selected;
                m_rotationCenterSet = true;
                m_selectingRotationCenter = false;
                m_hasHoveredPoint = false;
                setCursor(Qt::ArrowCursor);
                qDebug() << "Rotation center set at:" << m_rotationCenter;
                update();
            }
        } else {
            m_rotating = true;
        }
    } else if (event->button() == Qt::RightButton) {
        m_panning = true;
    }

    emit clicked();
}

QVector3D OpenGLWidget::findFallbackFBXPickPoint(const QPoint& screenPos, bool& pointFound)
{
    const float MAX_SCREEN_DISTANCE = 20.0f;
    float minDistSq = MAX_SCREEN_DISTANCE * MAX_SCREEN_DISTANCE;

    QVector3D closestPoint;
    QVector3D fallbackCenter(0, 0, 0);
    pointFound = false;
    bool hasFallback = false;

    makeCurrent();

    GLint viewport[4];
    GLdouble modelview[16];
    GLdouble projection[16];
    glGetIntegerv(GL_VIEWPORT, viewport);
    glGetDoublev(GL_MODELVIEW_MATRIX, modelview);
    glGetDoublev(GL_PROJECTION_MATRIX, projection);

    for (const auto& model : m_models) {
        if (!model.visible || model.meshes.empty()) continue;

        QMatrix4x4 transform;
        transform.translate(model.offset);
        transform.rotate(model.rotation.x(), 1, 0, 0);
        transform.rotate(model.rotation.y(), 0, 1, 0);
        transform.rotate(model.rotation.z(), 0, 0, 1);
        transform.scale(model.scale);

        for (const auto& mesh : model.meshes) {
            if (mesh.vertices.empty() || mesh.indices.size() < 3) continue;

            // Try triangle center projection picking
            for (size_t i = 0; i + 2 < mesh.indices.size(); i += 3) {
                QVector3D v1 = transform.map(mesh.vertices[mesh.indices[i]]);
                QVector3D v2 = transform.map(mesh.vertices[mesh.indices[i + 1]]);
                QVector3D v3 = transform.map(mesh.vertices[mesh.indices[i + 2]]);
                QVector3D triCenter = (v1 + v2 + v3) / 3.0f;

                GLdouble sx, sy, sz;
                if (!gluProject(triCenter.x(), triCenter.y(), triCenter.z(),
                                modelview, projection, viewport,
                                &sx, &sy, &sz)) continue;

                float dx = screenPos.x() - float(sx);
                float dy = (viewport[3] - screenPos.y()) - float(sy);
                float distSq = dx * dx + dy * dy;

                if (distSq < minDistSq) {
                    minDistSq = distSq;
                    closestPoint = triCenter;
                    pointFound = true;
                }
            }

            // Fallback: bounding box center of mesh part
            if (!hasFallback) {
                QVector3D minPt = mesh.vertices[0];
                QVector3D maxPt = mesh.vertices[0];
                for (const auto& v : mesh.vertices) {
                    minPt.setX(std::min(minPt.x(), v.x()));
                    minPt.setY(std::min(minPt.y(), v.y()));
                    minPt.setZ(std::min(minPt.z(), v.z()));
                    maxPt.setX(std::max(maxPt.x(), v.x()));
                    maxPt.setY(std::max(maxPt.y(), v.y()));
                    maxPt.setZ(std::max(maxPt.z(), v.z()));
                }
                fallbackCenter = transform.map((minPt + maxPt) * 0.5f);
                hasFallback = true;
            }
        }
    }

    if (!pointFound && hasFallback) {
        closestPoint = fallbackCenter;
        pointFound = true;
        qDebug() << "[FBX Pick] Using fallback center:" << closestPoint;
    } else if (pointFound) {
        qDebug() << "[FBX Pick] Found projected triangle center:" << closestPoint;
    } else {
        qDebug() << "[FBX Pick] No surface point or fallback found.";
    }

    return closestPoint;
}

QVector3D OpenGLWidget::findClosestFBXVertex(const QPoint& screenPos, bool& pointFound)
{
    const float MAX_SCREEN_DIST_SQ = 16.0f; // 4px radius
    QVector3D closestPoint;
    float minDistanceSq = MAX_SCREEN_DIST_SQ;
    pointFound = false;

    makeCurrent();

    GLint viewport[4];
    GLdouble modelview[16], projection[16];
    glGetIntegerv(GL_VIEWPORT, viewport);
    glGetDoublev(GL_MODELVIEW_MATRIX, modelview);
    glGetDoublev(GL_PROJECTION_MATRIX, projection);

    for (const auto& model : m_models) {
        if (!model.visible) continue;

        QMatrix4x4 modelTransform;
        modelTransform.translate(model.offset);
        modelTransform.rotate(model.rotation.x(), 1, 0, 0);
        modelTransform.rotate(model.rotation.y(), 0, 1, 0);
        modelTransform.rotate(model.rotation.z(), 0, 0, 1);
        modelTransform.scale(model.scale);

        for (const auto& node : model.nodes) {
            if (!node.visible) continue;

            QMatrix4x4 nodeTransform;
            nodeTransform.translate(node.position);
            nodeTransform.rotate(node.rotation.x(), 1, 0, 0);
            nodeTransform.rotate(node.rotation.y(), 0, 1, 0);
            nodeTransform.rotate(node.rotation.z(), 0, 0, 1);
            nodeTransform.scale(node.scale);

            QMatrix4x4 fullTransform = modelTransform * nodeTransform;

            for (const auto& mesh : node.meshes) {
                for (const auto& vertex : mesh.vertices) {
                    QVector3D world = fullTransform.map(vertex);

                    GLdouble sx, sy, sz;
                    if (!gluProject(world.x(), world.y(), world.z(),
                                    modelview, projection, viewport,
                                    &sx, &sy, &sz)) continue;

                    float dx = screenPos.x() - float(sx);
                    float dy = (viewport[3] - screenPos.y()) - float(sy);
                    float distSq = dx * dx + dy * dy;

                    if (distSq < minDistanceSq) {
                        minDistanceSq = distSq;
                        closestPoint = world;
                        pointFound = true;

                        if (distSq < 4.0f)  // Very close match: early exit
                            return closestPoint;
                    }
                }
            }
        }
    }

    return closestPoint;
}

// --- Refined FBX Debug Point Rendering with Screen-Space Hover Detection ---
void OpenGLWidget::drawFBXDebugPoints()
{
    if (!m_selectingRotationCenter) return;

    makeCurrent();

    GLint viewport[4];
    GLdouble modelview[16];
    GLdouble projection[16];
    glGetIntegerv(GL_VIEWPORT, viewport);
    glGetDoublev(GL_MODELVIEW_MATRIX, modelview);
    glGetDoublev(GL_PROJECTION_MATRIX, projection);

    glDisable(GL_LIGHTING);
    glEnable(GL_POINT_SMOOTH);
    glDepthFunc(GL_ALWAYS);
    glBegin(GL_POINTS);

    for (const auto& model : m_models) {
        if (!model.visible) continue;

        QMatrix4x4 modelTransform;
        modelTransform.translate(model.offset);
        modelTransform.rotate(model.rotation.x(), 1, 0, 0);
        modelTransform.rotate(model.rotation.y(), 0, 1, 0);
        modelTransform.rotate(model.rotation.z(), 0, 0, 1);
        modelTransform.scale(model.scale);

        for (const auto& node : model.nodes) {
            if (!node.visible) continue;

            QMatrix4x4 nodeTransform;
            nodeTransform.translate(node.position);
            nodeTransform.rotate(node.rotation.x(), 1, 0, 0);
            nodeTransform.rotate(node.rotation.y(), 0, 1, 0);
            nodeTransform.rotate(node.rotation.z(), 0, 0, 1);
            nodeTransform.scale(node.scale);

            QMatrix4x4 fullTransform = modelTransform * nodeTransform;

            for (const auto& mesh : node.meshes) {
                for (const auto& vertex : mesh.vertices) {
                    QVector3D world = fullTransform.map(vertex);

                    GLdouble sx, sy, sz;
                    if (!gluProject(world.x(), world.y(), world.z(), modelview, projection, viewport, &sx, &sy, &sz))
                        continue;

                    float dx = m_mouseScreenPos.x() - float(sx);
                    float dy = m_mouseScreenPos.y() - float(sy);
                    float screenDistSq = dx * dx + dy * dy;

                    if (m_hasHoveredPoint && screenDistSq < 36.0f) {
                        glColor3f(1.0f, 1.0f, 0.0f);  // Yellow
                        glPointSize(26.0f);
                    } else {
                        glColor3f(0.2f, 1.0f, 0.2f);
                        glPointSize(6.0f);
                    }
                    glVertex3f(world.x(), world.y(), world.z());
                }
            }
        }
    }

    glEnd();
    glDepthFunc(GL_LESS);
    glDisable(GL_POINT_SMOOTH);
    glEnable(GL_LIGHTING);
}
void OpenGLWidget::applyTransformToModel(const QString& name, const QVector3D& position, const QVector3D& rotation, const QVector3D& scale) {
    for (auto& model : m_models) {
        if (model.name == name) {
            model.position = position;
            model.rotation = rotation;
            model.scale = scale;
            emit modelTransformChanged(name);
            update();
            break;
        }
    }
}
void OpenGLWidget::updatePointCloudColors(const QString& name, const std::vector<QVector3D>& newColors) 
{
    qDebug() << "Updating point cloud colors for:" << name << "with" << newColors.size() << "colors";
    
    makeCurrent();
    
    for (auto& pc : m_pointClouds) {
        if (pc.name == name) {
            if (pc.labels.empty()) {
                qDebug() << "Error: No labels provided for point cloud" << name;
                doneCurrent();
                return;
            }
            
            if (newColors.empty()) {
                qDebug() << "Error: No colors provided for point cloud" << name;
                doneCurrent();
                return;
            }
            
            qDebug() << "Point cloud found. Points:" << pc.points.size() << "Labels:" << pc.labels.size();
            
            // Find the maximum cluster label to understand the mapping
            int maxLabel = -1;
            std::set<int> validLabels;
            for (int label : pc.labels) {
                if (label >= 0) {
                    validLabels.insert(label);
                    maxLabel = std::max(maxLabel, label);
                }
            }
            
            qDebug() << "Found" << validLabels.size() << "unique cluster labels, max label:" << maxLabel;
            
            // Ensure the colors vector is sized for all points
            pc.colors.resize(pc.points.size(), QVector3D(0.5f, 0.5f, 0.5f)); // Default to gray
            
            // Create label to color index mapping (sorted labels map to sequential color indices)
            std::map<int, int> labelToColorIndex;
            int colorIndex = 0;
            for (int label : validLabels) {
                labelToColorIndex[label] = colorIndex++;
            }
            
            // Map cluster colors to points based on labels
            int coloredPoints = 0;
            int noisePoints = 0;
            int invalidPoints = 0;
            
            for (size_t i = 0; i < pc.points.size(); ++i) {
                int label = pc.labels[i];
                
                if (label >= 0) {
                    // Valid cluster label - find corresponding color
                    auto it = labelToColorIndex.find(label);
                    if (it != labelToColorIndex.end() && it->second < static_cast<int>(newColors.size())) {
                        pc.colors[i] = newColors[it->second];
                        coloredPoints++;
                    } else {
                        // Label exists but no corresponding color
                        pc.colors[i] = QVector3D(0.7f, 0.7f, 0.7f); // Light gray
                        invalidPoints++;
                        qDebug() << "Warning: No color for valid label" << label << "at point" << i;
                    }
                } else if (label == -1) {
                    // Noise point - assign dark gray color
                    pc.colors[i] = QVector3D(0.3f, 0.3f, 0.3f);
                    noisePoints++;
                } else {
                    // Invalid label - assign different gray
                    pc.colors[i] = QVector3D(0.7f, 0.7f, 0.7f);
                    invalidPoints++;
                    qDebug() << "Warning: Invalid label" << label << "at point" << i;
                }
            }
            
            qDebug() << "Color mapping complete. Colored points:" << coloredPoints 
                     << "Noise points:" << noisePoints 
                     << "Invalid points:" << invalidPoints
                     << "Total points:" << pc.points.size();
            
            // Update the VBO with the new colors
            updatePointCloudVBO(pc);
            break;
        }
    }
    
    doneCurrent();
    update();
    
    qDebug() << "Point cloud color update completed";
}

// Add this new overloaded method to handle the cluster color map
void OpenGLWidget::updatePointCloudColorsWithMap(const QString& name, const std::map<int, QVector3D>& clusterColorMap) 
{
    qDebug() << "Updating point cloud colors with cluster map for:" << name << "with" << clusterColorMap.size() << "cluster colors";
    
    makeCurrent();
    
    for (auto& pc : m_pointClouds) {
        if (pc.name == name) {
            if (pc.labels.empty()) {
                qDebug() << "Error: No labels provided for point cloud" << name;
                doneCurrent();
                return;
            }
            
            if (clusterColorMap.empty()) {
                qDebug() << "Error: No cluster colors provided for point cloud" << name;
                doneCurrent();
                return;
            }
            
            qDebug() << "Point cloud found. Points:" << pc.points.size() << "Labels:" << pc.labels.size();
            
            // Ensure the colors vector is sized for all points
            pc.colors.resize(pc.points.size(), QVector3D(0.5f, 0.5f, 0.5f)); // Default to gray
            
            // Apply colors based on cluster labels
            int coloredPoints = 0;
            int noisePoints = 0;
            int unmappedPoints = 0;
            
            for (size_t i = 0; i < pc.points.size(); ++i) {
                int label = pc.labels[i];
                
                if (label >= 0) {
                    // Valid cluster label - look up color in map
                    auto colorIt = clusterColorMap.find(label);
                    if (colorIt != clusterColorMap.end()) {
                        pc.colors[i] = colorIt->second;
                        coloredPoints++;
                    } else {
                        // Valid label but no color mapping - use default
                        pc.colors[i] = QVector3D(0.8f, 0.8f, 0.8f); // Light gray
                        unmappedPoints++;
                    }
                } else if (label == -1) {
                    // Noise point - assign dark gray color
                    pc.colors[i] = QVector3D(0.3f, 0.3f, 0.3f);
                    noisePoints++;
                } else {
                    // Invalid label - assign red color for debugging
                    pc.colors[i] = QVector3D(1.0f, 0.0f, 0.0f);
                    unmappedPoints++;
                    qDebug() << "Warning: Invalid label" << label << "at point" << i;
                }
            }
            
            qDebug() << "Color mapping complete. Colored points:" << coloredPoints 
                     << "Noise points:" << noisePoints 
                     << "Unmapped points:" << unmappedPoints
                     << "Total points:" << pc.points.size();
            
            // Update the VBO with the new colors
            updatePointCloudVBO(pc);
            break;
        }
    }
    
    doneCurrent();
    update();
    
    qDebug() << "Point cloud color update with map completed";
}

// Helper method to update point cloud data (if you need it)
void OpenGLWidget::updatePointCloudData(const PointCloud& updatedPC)
{
    qDebug() << "Updating point cloud data for:" << updatedPC.name;
    
    makeCurrent();
    
    for (auto& pc : m_pointClouds) {
        if (pc.name == updatedPC.name) {
            pc = updatedPC; // Copy the updated data
            updatePointCloudVBO(pc);
            break;
        }
    }
    
    doneCurrent();
    update();
}
QVector3D OpenGLWidget::applyTransformations(const QVector3D& point, 
                                           const QVector3D& offset, 
                                           const QVector3D& rotation, 
                                           const QVector3D& scale) const
{
    QMatrix4x4 transform;
    
    // Apply scale
    transform.scale(scale);
    
    // Apply rotation (in degrees)
    transform.rotate(rotation.x(), 1.0f, 0.0f, 0.0f);
    transform.rotate(rotation.y(), 0.0f, 1.0f, 0.0f);
    transform.rotate(rotation.z(), 0.0f, 0.0f, 1.0f);
    
    // Apply translation (offset)
    transform.translate(offset);
    
    return transform * point;
}
void OpenGLWidget::removePointCloud(const QString& name)
{
    auto it = std::find_if(m_pointClouds.begin(), m_pointClouds.end(),
                           [&name](const PointCloud& pc) { return pc.name == name; });
    if (it != m_pointClouds.end()) {
        // Clean up OpenGL resources
        if (it->vboInitialized && it->vbo != 0) {
            glDeleteBuffers(1, &it->vbo);
        }
        m_pointClouds.erase(it);
        m_viewChanged = true; // Mark view as changed to trigger redraw
        update();
        qDebug() << "Removed PointCloud:" << name;
    }
}

void OpenGLWidget::removeModel(const QString& name)
{
    auto it = std::find_if(m_models.begin(), m_models.end(),
                           [&name](const Model& model) { return model.name == name; });
    if (it != m_models.end()) {
        // Clean up OpenGL resources for each mesh
        for (auto& mesh : it->meshes) {
            if (mesh.vboInitialized) {
                if (mesh.vbo != 0) glDeleteBuffers(1, &mesh.vbo);
                if (mesh.ibo != 0) glDeleteBuffers(1, &mesh.ibo);
            }
        }
        m_models.erase(it);
        m_viewChanged = true; // Mark view as changed to trigger redraw
        update();
        qDebug() << "Removed Model:" << name;
    }
}

void OpenGLWidget::remove2DGrid(const QString& name)
{
    auto it = std::find_if(m_grids2D.begin(), m_grids2D.end(),
                           [&name](const Grid2D& grid) { return grid.name == name; });
    if (it != m_grids2D.end()) {
        // Clean up OpenGL resources
        if (it->vboInitialized) {
            if (it->vbo != 0) glDeleteBuffers(1, &it->vbo);
            if (it->ibo != 0) glDeleteBuffers(1, &it->ibo);
        }
        m_grids2D.erase(it);
        m_viewChanged = true; // Mark view as changed to trigger redraw
        update();
        qDebug() << "Removed 2DGrid:" << name;
    }
}

void OpenGLWidget::remove3DGrid(const QString& name)
{
    auto it = std::find_if(m_grids3D.begin(), m_grids3D.end(),
                           [&name](const Grid3D& grid) { return grid.name == name; });
    if (it != m_grids3D.end()) {
        // Clean up OpenGL resources
        if (it->vboInitialized) {
            if (it->vbo != 0) glDeleteBuffers(1, &it->vbo);
            if (it->ibo != 0) glDeleteBuffers(1, &it->ibo);
        }
        m_grids3D.erase(it);
        m_viewChanged = true; // Mark view as changed to trigger redraw
        update();
        qDebug() << "Removed 3DGrid:" << name;
    }
}