// viewportcapturedialog.h
#ifndef VIEWPORTCAPTUREDIALOG_H
#define VIEWPORTCAPTUREDIALOG_H

#include <QDialog>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QFormLayout>
#include <QDialogButtonBox>
#include <QDoubleSpinBox>
#include <QLineEdit>
#include <QLabel>
#include <QComboBox>
#include <QVector3D>
#include <QString>
#include <QGroupBox>

class ViewportCaptureDialog : public QDialog
{
    Q_OBJECT

public:
    explicit ViewportCaptureDialog(QWidget *parent = nullptr);
    ~ViewportCaptureDialog();

    // Get methods to retrieve user inputs
    QVector3D getCameraPosition() const;
    QVector3D getCameraTarget() const;
    QVector3D getCameraUp() const;
    QString getCaptureName() const;

    // Set methods to initialize with current values
    void setCameraPosition(const QVector3D &position);
    void setCameraTarget(const QVector3D &target);
    void setCameraUp(const QVector3D &up);
    void setPresetOptions(const QStringList &presets);

private slots:
    void onPresetSelected(int index);

private:
    QLineEdit *nameEdit;
    QDoubleSpinBox *posXSpin, *posYSpin, *posZSpin;
    QDoubleSpinBox *targetXSpin, *targetYSpin, *targetZSpin;
    QDoubleSpinBox *upXSpin, *upYSpin, *upZSpin;
    QComboBox *presetComboBox;

    void setupUI();
};

#endif // VIEWPORTCAPTUREDIALOG_H
