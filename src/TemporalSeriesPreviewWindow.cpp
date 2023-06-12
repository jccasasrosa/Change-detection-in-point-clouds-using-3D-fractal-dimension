#include "temporalseriespreviewwindow.h"
#include <QIntValidator>

TemporalSeriesPreviewWindow::TemporalSeriesPreviewWindow(QWidget *parent)
    : QDialog(parent)
    , ui(new Ui::TemporalSeriesPreviewWindow)
{
    ui->setupUi(this);
    setWindowFlags(windowFlags() & ~Qt::WindowContextHelpButtonHint);
    ui->lineEdit->setValidator(new QIntValidator(1, 200, this));
    ui->lineEdit_2->setValidator(new QIntValidator(1, 20000, this));
    ui->lineEdit_3->setValidator(new QIntValidator(1, 20, this));
}

TemporalSeriesPreviewWindow::~TemporalSeriesPreviewWindow()
{
    delete ui;
}

int TemporalSeriesPreviewWindow::getNumberOfCells()
{
    return ui->lineEdit->text().toInt();
}

int TemporalSeriesPreviewWindow::getMinimumOctreeNodePoints()
{
    return ui->lineEdit_2->text().toInt();
}

int TemporalSeriesPreviewWindow::getMaximumOctreeDepthLevel()
{
    return ui->lineEdit_3->text().toInt();
}

bool TemporalSeriesPreviewWindow::visualizationChecked()
{
    return ui->checkBox->isChecked();
}

void TemporalSeriesPreviewWindow::setDefaultNumberOfCells(int number)
{
    ui->lineEdit->setText(QString::number(number));
}

void TemporalSeriesPreviewWindow::setDefaultMinimumOctreeNodePoints(int number)
{
    ui->lineEdit_2->setText(QString::number(number));
}

void TemporalSeriesPreviewWindow::setDefaultMaximumOctreeDepthLevel(int number)
{
    ui->lineEdit_3->setText(QString::number(number));
}

void TemporalSeriesPreviewWindow::cancelButton()
{
    reject();
}

void TemporalSeriesPreviewWindow::acceptButton()
{
    accept();
}

