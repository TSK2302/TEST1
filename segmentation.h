#ifndef SEGMENTATION_H
#define SEGMENTATION_H

#include <QDialog>
#include <QTableWidgetItem>

class OpenGLWidget; // Forward declaration

namespace Ui {
class SegmentationDialog;
}

class SegmentationDialog : public QDialog
{
    Q_OBJECT

public:
    explicit SegmentationDialog(OpenGLWidget* openGLWidget, QWidget *parent = nullptr);
    ~SegmentationDialog();

private slots:
    void on_buttonBox_accepted();
    void on_buttonBox_rejected();
    void on_tableWidget_itemChanged(QTableWidgetItem *item);
    void on_clearSelections_clicked();

private:
    Ui::SegmentationDialog *ui;
    OpenGLWidget* openGLWidget;
    bool isInitializing;  // Added flag to track initialization state

    void populateTable();
    void clearSelections(const QString& gridName);
    void updateScene(); // Helper to ensure scene updates
};

#endif // SEGMENTATION_H