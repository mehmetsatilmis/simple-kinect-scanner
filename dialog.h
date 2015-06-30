#ifndef DIALOG_H
#define DIALOG_H

#include <QDialog>

namespace Ui {
class Dialog;
}

class Dialog : public QDialog
{
    Q_OBJECT
    
public:
    explicit Dialog(QWidget *parent = 0);
    ~Dialog();
    double getX();
    double getY();
    double getZ();

    
private slots:
    void on_pushButton_clicked();

private:
    Ui::Dialog *ui;
    double pre_x,pre_y,pre_z;
    bool isFirstTime;
};

#endif // DIALOG_H
