// #ifdef min
// #undef min
// #endif
// #ifdef max
// #undef max
// #endif
// #include "mainwindow.h"

// #include <QApplication>

// int main(int argc, char *argv[])
// {

//     QApplication a(argc, argv);
//     MainWindow w;
//     w.show();
//     return a.exec();
// }
#ifdef min
#undef min
#endif
#ifdef max
#undef max
#endif

#include "mainwindow.h"
#include <QApplication>
#include <QFontDatabase>
#include <QFont>
#include <QDebug>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

    // Load custom fonts
    QStringList fontPaths = {
        ":/fonts/Roboto-Regular.ttf",
        ":/fonts/Roboto-Bold.ttf",
        ":/fonts/Roboto-Light.ttf",
        ":/fonts/Roboto-Medium.ttf"
    };

    QString fontFamily = "Roboto"; // Default fallback

    for (const QString& path : fontPaths) {
        int fontId = QFontDatabase::addApplicationFont(path);
        if (fontId != -1) {
            QStringList families = QFontDatabase::applicationFontFamilies(fontId);
            if (!families.isEmpty()) {
                fontFamily = families.at(0);
                qDebug() << "Loaded font:" << path << "Family:" << fontFamily;
            }
        } else {
            qDebug() << "Failed to load font:" << path;
        }
    }

    // Set application default font
    QFont appFont(fontFamily, 10, QFont::Normal);
    a.setFont(appFont);

    MainWindow w;
    w.show();

    return a.exec();
}
