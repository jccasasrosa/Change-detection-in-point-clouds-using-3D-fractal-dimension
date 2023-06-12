#ifndef TEMPORALSERIESPREVIEWWINDOW_H
#define TEMPORALSERIESPREVIEWWINDOW_H

#include <QDialog>
#include "ui_temporalseriespreviewwindow.h"

QT_BEGIN_NAMESPACE
namespace Ui { class TemporalSeriesPreviewWindow; }
QT_END_NAMESPACE

class TemporalSeriesPreviewWindow : public QDialog, public Ui::TemporalSeriesPreviewWindow
{
    Q_OBJECT

public:
    explicit TemporalSeriesPreviewWindow(QWidget *parent = nullptr);
    ~TemporalSeriesPreviewWindow();

    int getNumberOfCells();
    int getMinimumOctreeNodePoints();
    int getMaximumOctreeDepthLevel();
    bool visualizationChecked();

    void setDefaultNumberOfCells(int number);
    void setDefaultMinimumOctreeNodePoints(int number);
    void setDefaultMaximumOctreeDepthLevel(int number);

private:
    Ui::TemporalSeriesPreviewWindow *ui;

private slots:
    void cancelButton();
    void acceptButton();
    
};
#endif // TEMPORALSERIESPREVIEWWINDOW_H
