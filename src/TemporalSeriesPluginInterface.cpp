#include "TemporalSeriesPluginInterface.h"

TemporalSeriesPluginInterface::TemporalSeriesPluginInterface(QWidget* parent)
    : ccOverlayDialog(parent)
    , ui(new Ui::TemporalSeriesPluginInterface)
{
    setupUi(this);
}

TemporalSeriesPluginInterface::~TemporalSeriesPluginInterface()
{
}

