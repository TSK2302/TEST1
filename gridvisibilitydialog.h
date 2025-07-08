#ifndef GRIDVISIBILITYDIALOG_H
#define GRIDVISIBILITYDIALOG_H

#include <QDialog>
#include <QCheckBox>
#include <vector>
#include "openglwidget.h"

namespace Ui {
class GridVisibilityDialog;
}

class GridVisibilityDialog : public QDialog
{
    Q_OBJECT

public:
    explicit GridVisibilityDialog(OpenGLWidget* openGLWidget, const std::vector<OpenGLWidget::Grid2D>& grids2D, QWidget *parent = nullptr);
    ~GridVisibilityDialog();

signals:
    void gridVisibilityChanged(const QString& name, bool visible);

private slots:
    void on_buttonBox_accepted();
    void on_buttonBox_rejected();
    void on_checkBox_stateChanged(int state);

private:
    Ui::GridVisibilityDialog *ui;
    std::vector<QCheckBox*> m_checkBoxes;
    std::vector<OpenGLWidget::Grid2D> m_grids2D;
    OpenGLWidget* m_openGLWidget; // Pointer to OpenGLWidget
};

#endif // GRIDVISIBILITYDIALOG_H