#ifndef IDENTIFYSIMILARCLUSTERSDIALOG_H
#define IDENTIFYSIMILARCLUSTERSDIALOG_H

#include <QDialog>
#include <QVBoxLayout>
#include <QLabel>
#include <QDialogButtonBox>
#include <QPushButton>

class IdentifySimilarClustersDialog : public QDialog
{
    Q_OBJECT

public:
    explicit IdentifySimilarClustersDialog(QWidget *parent = nullptr);

signals:
    void identifySimilarClustersConfirmed();

private slots:
    void onAccept();
};

#endif // IDENTIFYSIMILARCLUSTERSDIALOG_H