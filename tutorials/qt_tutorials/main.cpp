#include <QApplication>
#include <QLabel>
#include <QPushButton>
#include <weikasmainwindow.h>
#include <QSlider>
#include <QSpinBox>
#include <QHBoxLayout>

int main(int argc, char *argv[])
{
    QApplication prog(argc, argv);

    WeikasMainWindow w;
    w.show();

//    QWidget *mainWindow = new QWidget;
//    mainWindow->setWindowTitle("Toolbox for Multi Sensor Data Fusion");

//    QLabel *label = new QLabel("Gametime");
//    QPushButton *button = new QPushButton("Quit the window");
//    QObject::connect(button, SIGNAL(clicked()), &prog, SLOT(quit()));


//    QSpinBox *spinner = new QSpinBox;
//    QSlider *slider = new QSlider;
//    spinner->setRange(1,50);
//    slider->setRange(1,50);
//    QObject::connect(spinner, SIGNAL(valueChanged(int)), slider, SLOT(setValue(int)));
//    QObject::connect(slider, SIGNAL(valueChanged(int)), spinner, SLOT(setValue(int)));
//    spinner->setValue(10);

//    QHBoxLayout *layout = new QHBoxLayout;
//    layout->addWidget(slider);
//    layout->addWidget(spinner);
//    layout->addWidget(button);
//    layout->addWidget(label);

//    mainWindow->setLayout(layout);
//    mainWindow->show();

    return prog.exec();
}
