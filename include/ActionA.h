// Example of a plugin action

#pragma once
#include <glm.hpp>
#include <TemporalSeriesPluginInterface.h>
#include <TemporalSeriesPlugin.h>
#include "OctreeGrid.h"

class ccMainAppInterface;

namespace TemporalSeriesPluginAction
{
	static OctreeGrid* _spatialStructure;
	void performActionA( ccMainAppInterface *appInterface, TemporalSeriesPluginInterface* temporalSeriesPluginInterface);
	void drawBoundingBox(const std::vector<glm::vec3>& boundingBox, ccMainAppInterface* appInterface, const ccColor::Rgba& color, const bool& enable);
	void octreeButtonAction(ccMainAppInterface* appInterface);
	void nodeButtonAction(ccMainAppInterface* appInterface);
}
