#include "weikasmainwindow.h"
#include "ui_weikasmainwindow.h"
#include <QtCore/QFile>
#include <QtCore/QTextStream>

WeikasMainWindow::WeikasMainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::WeikasMainWindow)
{
    ui->setupUi(this);
    getTextFile();
}

WeikasMainWindow::~WeikasMainWindow()
{
    delete ui;
}

void WeikasMainWindow::on_goButton_clicked()
{
    //ui->goButton->setText("Searching!!");
    QString word = ui->lineEdit->text();
    ui->textEdit->find(word, QTextDocument::FindWholeWords);
}

void WeikasMainWindow::getTextFile()
{
    QFile myFile(":/README.txt");
    myFile.open(QIODevice::ReadOnly);
    QTextStream textStream(&myFile);
    QString line = textStream.readAll();
    myFile.close();

    ui->textEdit->setPlainText(line);
    QTextCursor textCursor = ui->textEdit->textCursor();
    textCursor.movePosition(QTextCursor::Start, QTextCursor::MoveAnchor, 1);
}

void WeikasMainWindow::on_horizontalSlider_valueChanged(int value)
{
    ui->spinBox->setValue(ui->horizontalSlider->value());
}

void WeikasMainWindow::on_spinBox_valueChanged(int arg1)
{
    ui->horizontalSlider->setValue(ui->spinBox->value());
}

void WeikasMainWindow::on_cancelButtonBox_clicked(QAbstractButton *button)
{
    ui->weikaslabel1->setText("Ouch: Please dont cancel!");
}
