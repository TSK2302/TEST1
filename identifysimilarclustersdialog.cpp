#include "identifysimilarclustersdialog.h"

IdentifySimilarClustersDialog::IdentifySimilarClustersDialog(QWidget *parent)
    : QDialog(parent)
{
    setWindowTitle(tr("Identify Similar Clusters"));
    setMinimumWidth(300);

    QVBoxLayout *mainLayout = new QVBoxLayout(this);

    // Information label
    QLabel *infoLabel = new QLabel(tr("This will identify similar clusters based on bounding box dimensions and point counts."));
    infoLabel->setWordWrap(true);
    mainLayout->addWidget(infoLabel);

    // Buttons
    QDialogButtonBox *buttonBox = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
    connect(buttonBox, &QDialogButtonBox::accepted, this, &IdentifySimilarClustersDialog::onAccept);
    connect(buttonBox, &QDialogButtonBox::rejected, this, &QDialog::reject);
    mainLayout->addWidget(buttonBox);

    setLayout(mainLayout);
}

void IdentifySimilarClustersDialog::onAccept()
{
    emit identifySimilarClustersConfirmed();
    accept();
}