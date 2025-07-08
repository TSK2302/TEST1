#include "viewportcapturedialog.h"
#include <QDateTime>
#include <QGroupBox>

ViewportCaptureDialog::ViewportCaptureDialog(QWidget *parent)
    : QDialog(parent)
{
    setWindowTitle(tr("User Viewport Capture"));
    setMinimumWidth(450);
    setupUI();

    // Generate a default name with timestamp
    QDateTime now = QDateTime::currentDateTime();
    nameEdit->setText(QString("UserViewport_%1").arg(now.toString("yyyyMMdd_hhmmss")));
}

ViewportCaptureDialog::~ViewportCaptureDialog()
{
}

void ViewportCaptureDialog::setupUI()
{
    QVBoxLayout *mainLayout = new QVBoxLayout(this);

    // Name section
    QFormLayout *nameLayout = new QFormLayout();
    nameEdit = new QLineEdit(this);
    nameLayout->addRow(tr("Capture Name:"), nameEdit);
    mainLayout->addLayout(nameLayout);

    // Preset section
    QFormLayout *presetLayout = new QFormLayout();
    presetComboBox = new QComboBox(this);
    presetComboBox->addItem(tr("Custom"));
    presetComboBox->addItem(tr("Front View"));
    presetComboBox->addItem(tr("Top View"));
    presetComboBox->addItem(tr("Side View"));
    presetComboBox->addItem(tr("Isometric View"));
    presetLayout->addRow(tr("Preset View:"), presetComboBox);
    mainLayout->addLayout(presetLayout);

    // Camera Position group
    QGroupBox *posGroup = new QGroupBox(tr("Camera Position"));
    QFormLayout *posLayout = new QFormLayout(posGroup);

    posXSpin = new QDoubleSpinBox(this);
    posYSpin = new QDoubleSpinBox(this);
    posZSpin = new QDoubleSpinBox(this);

    // Configure the spin boxes for large range of values
    for (QDoubleSpinBox *spin : {posXSpin, posYSpin, posZSpin}) {
        spin->setRange(-10000.0, 10000.0);
        spin->setDecimals(3);
        spin->setSingleStep(1.0);
    }

    QHBoxLayout *posXYZLayout = new QHBoxLayout();
    posXYZLayout->addWidget(new QLabel("X:"));
    posXYZLayout->addWidget(posXSpin);
    posXYZLayout->addWidget(new QLabel("Y:"));
    posXYZLayout->addWidget(posYSpin);
    posXYZLayout->addWidget(new QLabel("Z:"));
    posXYZLayout->addWidget(posZSpin);

    posLayout->addRow(tr("Coordinates:"), posXYZLayout);
    mainLayout->addWidget(posGroup);

    // Camera Target group
    QGroupBox *targetGroup = new QGroupBox(tr("Camera Target (Look At)"));
    QFormLayout *targetLayout = new QFormLayout(targetGroup);

    targetXSpin = new QDoubleSpinBox(this);
    targetYSpin = new QDoubleSpinBox(this);
    targetZSpin = new QDoubleSpinBox(this);

    // Configure the spin boxes
    for (QDoubleSpinBox *spin : {targetXSpin, targetYSpin, targetZSpin}) {
        spin->setRange(-10000.0, 10000.0);
        spin->setDecimals(3);
        spin->setSingleStep(1.0);
    }

    QHBoxLayout *targetXYZLayout = new QHBoxLayout();
    targetXYZLayout->addWidget(new QLabel("X:"));
    targetXYZLayout->addWidget(targetXSpin);
    targetXYZLayout->addWidget(new QLabel("Y:"));
    targetXYZLayout->addWidget(targetYSpin);
    targetXYZLayout->addWidget(new QLabel("Z:"));
    targetXYZLayout->addWidget(targetZSpin);

    targetLayout->addRow(tr("Coordinates:"), targetXYZLayout);
    mainLayout->addWidget(targetGroup);

    // Camera Up vector group
    QGroupBox *upGroup = new QGroupBox(tr("Camera Up Vector"));
    QFormLayout *upLayout = new QFormLayout(upGroup);

    upXSpin = new QDoubleSpinBox(this);
    upYSpin = new QDoubleSpinBox(this);
    upZSpin = new QDoubleSpinBox(this);

    // Configure the spin boxes
    for (QDoubleSpinBox *spin : {upXSpin, upYSpin, upZSpin}) {
        spin->setRange(-1.0, 1.0);
        spin->setDecimals(3);
        spin->setSingleStep(0.1);
    }

    // Default up vector is (0, 1, 0)
    upXSpin->setValue(0.0);
    upYSpin->setValue(1.0);
    upZSpin->setValue(0.0);

    QHBoxLayout *upXYZLayout = new QHBoxLayout();
    upXYZLayout->addWidget(new QLabel("X:"));
    upXYZLayout->addWidget(upXSpin);
    upXYZLayout->addWidget(new QLabel("Y:"));
    upXYZLayout->addWidget(upYSpin);
    upXYZLayout->addWidget(new QLabel("Z:"));
    upXYZLayout->addWidget(upZSpin);

    upLayout->addRow(tr("Vector:"), upXYZLayout);
    mainLayout->addWidget(upGroup);

    // Button box
    QDialogButtonBox *buttonBox = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
    connect(buttonBox, &QDialogButtonBox::accepted, this, &QDialog::accept);
    connect(buttonBox, &QDialogButtonBox::rejected, this, &QDialog::reject);
    mainLayout->addWidget(buttonBox);

    // Connect preset combo box signal
    connect(presetComboBox, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &ViewportCaptureDialog::onPresetSelected);
}

QVector3D ViewportCaptureDialog::getCameraPosition() const
{
    return QVector3D(posXSpin->value(), posYSpin->value(), posZSpin->value());
}

QVector3D ViewportCaptureDialog::getCameraTarget() const
{
    return QVector3D(targetXSpin->value(), targetYSpin->value(), targetZSpin->value());
}

QVector3D ViewportCaptureDialog::getCameraUp() const
{
    return QVector3D(upXSpin->value(), upYSpin->value(), upZSpin->value());
}

QString ViewportCaptureDialog::getCaptureName() const
{
    return nameEdit->text();
}

void ViewportCaptureDialog::setCameraPosition(const QVector3D &position)
{
    posXSpin->setValue(position.x());
    posYSpin->setValue(position.y());
    posZSpin->setValue(position.z());
}

void ViewportCaptureDialog::setCameraTarget(const QVector3D &target)
{
    targetXSpin->setValue(target.x());
    targetYSpin->setValue(target.y());
    targetZSpin->setValue(target.z());
}

void ViewportCaptureDialog::setCameraUp(const QVector3D &up)
{
    upXSpin->setValue(up.x());
    upYSpin->setValue(up.y());
    upZSpin->setValue(up.z());
}

void ViewportCaptureDialog::setPresetOptions(const QStringList &presets)
{
    presetComboBox->clear();
    presetComboBox->addItem(tr("Custom"));
    presetComboBox->addItems(presets);
}

void ViewportCaptureDialog::onPresetSelected(int index)
{
    if (index == 0) {
        // Custom - do nothing, let user set values
        return;
    }

    // Set up standard views
    switch(index) {
    case 1: // Front View
        setCameraPosition(QVector3D(0, 0, 10));
        setCameraTarget(QVector3D(0, 0, 0));
        setCameraUp(QVector3D(0, 1, 0));
        break;

    case 2: // Top View
        setCameraPosition(QVector3D(0, 10, 0));
        setCameraTarget(QVector3D(0, 0, 0));
        setCameraUp(QVector3D(0, 0, -1));
        break;

    case 3: // Side View
        setCameraPosition(QVector3D(10, 0, 0));
        setCameraTarget(QVector3D(0, 0, 0));
        setCameraUp(QVector3D(0, 1, 0));
        break;

    case 4: // Isometric View
        setCameraPosition(QVector3D(7, 7, 7));
        setCameraTarget(QVector3D(0, 0, 0));
        setCameraUp(QVector3D(0, 1, 0));
        break;
    }
}
