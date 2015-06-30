#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "dialog.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT
    
public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private slots:



    void on_concatenate_clicked();

    void on_showMesh_clicked();

    void on_startScan_clicked();

    void on_crop_clicked();

    void on_showPoints_clicked();

private:
    Dialog m_dialog;
    Ui::MainWindow *ui;
};

#endif // MAINWINDOW_H
