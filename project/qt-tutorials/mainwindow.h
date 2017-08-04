#ifndef WEIKASMAINWINDOW_H
#define WEIKASMAINWINDOW_H

#include <QMainWindow>
#include <QAbstractButton>
namespace Ui {
class WeikasMainWindow;
}

class WeikasMainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit WeikasMainWindow(QWidget *parent = 0);
    ~WeikasMainWindow();

private slots:
    void on_goButton_clicked();

    void on_horizontalSlider_valueChanged(int value);

    void on_spinBox_valueChanged(int arg1);

    void on_cancelButtonBox_clicked(QAbstractButton *button);

private:
    Ui::WeikasMainWindow *ui;
    void getTextFile();
};

#endif // WEIKASMAINWINDOW_H
