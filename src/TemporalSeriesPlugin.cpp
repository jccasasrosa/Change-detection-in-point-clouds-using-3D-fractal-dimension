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

// First:
//	Replace all occurrences of 'ExamplePlugin' by your own plugin class name in this file.
//	This includes the resource path to info.json in the constructor.

// Second:
//	Open ExamplePlugin.qrc, change the "prefix" and the icon filename for your plugin.
//	Change the name of the file to <yourPluginName>.qrc

// Third:
//	Open the info.json file and fill in the information about the plugin.
//	 "type" should be one of: "Standard", "GL", or "I/O" (required)
//	 "name" is the name of the plugin (required)
//	 "icon" is the Qt resource path to the plugin's icon (from the .qrc file)
//	 "description" is used as a tootip if the plugin has actions and is displayed in the plugin dialog
//	 "authors", "maintainers", and "references" show up in the plugin dialog as well

#include <QtGui>

#include "TemporalSeriesPlugin.h"

#include "ActionA.h"

// Default constructor:
//	- pass the Qt resource path to the info.json file (from <yourPluginName>.qrc file) 
//  - constructor should mainly be used to initialize actions and other members
TemporalSeriesPlugin::TemporalSeriesPlugin( QObject *parent )
	: QObject( parent )
	, ccStdPluginInterface( ":/CC/plugin/TemporalSeriesPlugin/info.json" )
	, m_action( nullptr )
{

}

// This method should enable or disable your plugin actions
// depending on the currently selected entities ('selectedEntities').
void TemporalSeriesPlugin::onNewSelection( const ccHObject::Container &selectedEntities )
{
	if ( m_action == nullptr )
	{
		return;
	}
	
	m_action->setEnabled( true );
}

// This method returns all the 'actions' your plugin can perform.
// getActions() will be called only once, when plugin is loaded.
QList<QAction *> TemporalSeriesPlugin::getActions()
{
	// default action (if it has not been already created, this is the moment to do it)
	if ( !m_action )
	{
		m_action = new QAction( "Temporal Series Plugin", this );
		m_action->setToolTip( getDescription() );
		m_action->setIcon( getIcon() );
		

		_temporalSeriesPluginInterface = new TemporalSeriesPluginInterface((QWidget*)m_app->getMainWindow());

		/**
		_temporalSeriesPluginInterface->horizontalSlider->setMinimum(0);
		_temporalSeriesPluginInterface->horizontalSlider->setMaximum(8);
		_temporalSeriesPluginInterface->horizontalSlider->setValue(4);
		_temporalSeriesPluginInterface->horizontalSlider->setTickPosition(QSlider::TicksBothSides);
		*/

		// Connect appropriate signal
		connect( m_action, &QAction::triggered, this, [this]()
		{
			TemporalSeriesPluginAction::performActionA( m_app, _temporalSeriesPluginInterface );
		});

		connect(_temporalSeriesPluginInterface->toolButton, &QAbstractButton::toggled, this, [this]()
			{
				TemporalSeriesPluginAction::octreeButtonAction(m_app);
			});

		connect(_temporalSeriesPluginInterface->toolButton_2, &QAbstractButton::toggled, this, [this]()
			{
				TemporalSeriesPluginAction::nodeButtonAction(m_app);
			});

		/**
		connect(_temporalSeriesPluginInterface->horizontalSlider, &QAbstractSlider::valueChanged, this, [this]()
			{
				TemporalSeriesPluginAction::sliderChanged(m_app, _temporalSeriesPluginInterface->horizontalSlider);
			});
		*/
	}

	return { m_action };
}

