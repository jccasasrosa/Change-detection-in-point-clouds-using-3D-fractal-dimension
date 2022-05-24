#ifndef TEMPORALSERIESPLUGININTERFACE_H
#define TEMPORALSERIESPLUGININTERFACE_H

#include <QDialog>
#include <ccOverlayDialog.h>
#include "ui_temporalseriesplugininterface.h"

QT_BEGIN_NAMESPACE
namespace Ui { class TemporalSeriesPluginInterface; }
QT_END_NAMESPACE

class TemporalSeriesPluginInterface : public ccOverlayDialog, public Ui::TemporalSeriesPluginInterface
{
    Q_OBJECT

public:
    explicit TemporalSeriesPluginInterface(QWidget *parent = nullptr);
    ~TemporalSeriesPluginInterface();

private:
    Ui::TemporalSeriesPluginInterface *ui;
};
#endif // TEMPORALSERIESPLUGININTERFACE_H
