//##########################################################################
//#                                                                        #
//#                CLOUDCOMPARE PLUGIN: ExamplePlugin                      #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU General Public License as published by  #
//#  the Free Software Foundation; version 2 of the License.               #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#                             COPYRIGHT: XXX                             #
//#                                                                        #
//##########################################################################

#pragma once

#include "ccStdPluginInterface.h"
#include <TemporalSeriesPluginInterface.h>
#include <TemporalSeriesPreviewWindow.h>
#include <ccPointCloud.h>
#include <ccPolyline.h>


class TemporalSeriesPlugin : public QObject, public ccStdPluginInterface
{
	Q_OBJECT
	Q_INTERFACES( ccPluginInterface ccStdPluginInterface )
	Q_PLUGIN_METADATA( IID "cccorp.cloudcompare.plugin.TemporalSeriesPlugin" FILE "../info.json" )

public:
	explicit TemporalSeriesPlugin( QObject *parent = nullptr );
	~TemporalSeriesPlugin() override = default;

	// Inherited from ccStdPluginInterface
	void onNewSelection( const ccHObject::Container &selectedEntities ) override;
	QList<QAction *> getActions() override;

	void closeInterface();

	
private:

	QAction* m_action;
	TemporalSeriesPluginInterface* _temporalSeriesPluginInterface;
	TemporalSeriesPreviewWindow* _temporalSeriesPreviewWindow;
	ccHObject::Container m_selectedEntities;

};
