// Example of a plugin action

#include "ccMainAppInterface.h"
#include <qfiledialog.h>
#include "lasreader_las.hpp"
#include <ccPointCloud.h>
#include <ccScalarField.h>
#include <ccPolyline.h>
#include "qprogressdialog.h"
#include "qprogressbar.h"
#include "qapplication.h"
#include "..\include\ActionA.h"
#include <qelapsedtimer.h>


#define LOAD_INTERVAL 100000

namespace TemporalSeriesPluginAction
{

	void performActionA( ccMainAppInterface *appInterface, TemporalSeriesPluginInterface* temporalSeriesPluginInterface, TemporalSeriesPreviewWindow* temporalSeriesPreviewWindow)
	{
		if ( appInterface == nullptr )
		{
			// The application interface should have already been initialized when the plugin is loaded
			Q_ASSERT( false );
			
			return;
		}


		std::vector<ccPointCloud*> comparedClouds;

		const ccHObject::Container& selectedEntities = appInterface->getSelectedEntities();
		ccPointCloud* myPointCloud{};
		foreach(auto entity, selectedEntities) {
			comparedClouds.push_back(static_cast<ccPointCloud*>(entity));
			myPointCloud = static_cast<ccPointCloud*>(entity);
		}

		int numberOfCells, minimumOctreeNodePoints, maximumOctreeDepthLevel;
		float cellSize;
		bool visualizationChecked;

		//Default values
		numberOfCells = 1; //100
		minimumOctreeNodePoints = 1000; //10000
		maximumOctreeDepthLevel = 3; //6
		visualizationChecked = true;
		
		temporalSeriesPreviewWindow->setDefaultNumberOfCells(numberOfCells);
		temporalSeriesPreviewWindow->setDefaultMinimumOctreeNodePoints(minimumOctreeNodePoints);
		temporalSeriesPreviewWindow->setDefaultMaximumOctreeDepthLevel(maximumOctreeDepthLevel);
		int result = temporalSeriesPreviewWindow->exec();

		//QDialog::Accepted (1) or rejected (0)
		if (result == QDialog::Accepted) {

			appInterface->dispToConsole("[Temporal Series Plugin] Comparison accepted", ccMainAppInterface::STD_CONSOLE_MESSAGE);

			numberOfCells = temporalSeriesPreviewWindow->getNumberOfCells();
			minimumOctreeNodePoints = temporalSeriesPreviewWindow->getMinimumOctreeNodePoints();
			maximumOctreeDepthLevel = temporalSeriesPreviewWindow->getMaximumOctreeDepthLevel();
			visualizationChecked = temporalSeriesPreviewWindow->visualizationChecked();


			computePreviewParameters(&numberOfCells, &minimumOctreeNodePoints, &maximumOctreeDepthLevel, comparedClouds, &cellSize, appInterface);

			appInterface->dispToConsole(QString("[Counting Box] Parameters for the comparison: [%1 cells, %2 minimum octree node points, %3 maximum octree depth level] (%4 cell size) ").arg(numberOfCells).arg(minimumOctreeNodePoints).arg(maximumOctreeDepthLevel).arg(cellSize), ccMainAppInterface::STD_CONSOLE_MESSAGE);

			_spatialStructure = new OctreeGrid(cellSize, minimumOctreeNodePoints, maximumOctreeDepthLevel);
			_spatialStructure->setAppinterface(appInterface);

			//Definition of points of each file
			long totalPoints = comparedClouds[0]->size() + comparedClouds[1]->size();

			//Structure for retrieving times
			QElapsedTimer countingBoxTimer;
			qint64 distTime_ms = 0;
			countingBoxTimer.start();

			shiftCloudsOrigin(comparedClouds,appInterface);

			//Iterating through the clouds and generating the octree	
			for (int i = 0; i < comparedClouds.size(); ++i) {

				std::vector<glm::vec3> pointsPosition;
				std::vector<glm::vec3> pointsColor;


				int pointCloudSize = comparedClouds[i]->size();



				for (int j = 0; j < pointCloudSize; ++j) {

					float coordX = comparedClouds[i]->getPoint(j)->x;
					float coordY = comparedClouds[i]->getPoint(j)->y;
					float coordZ = comparedClouds[i]->getPoint(j)->z;

					int colorR = comparedClouds[i]->getPointColor(j).r;
					int colorG = comparedClouds[i]->getPointColor(j).g;
					int colorB = comparedClouds[i]->getPointColor(j).b;


					pointsPosition.push_back(glm::vec3(coordX, coordY, coordZ));
					pointsColor.push_back(glm::vec3(colorR, colorG, colorB));

				}


				_spatialStructure->loadOctreeGrid(pointsPosition, pointsColor, comparedClouds.size(), i);
				distTime_ms += countingBoxTimer.restart();

			}


			//Finished reading, completing subdivision of each octree in the octree grid
			countingBoxTimer.restart();
			_spatialStructure->completeSubdivide();
			distTime_ms += countingBoxTimer.restart();
			appInterface->dispToConsole(QString("[Counting Box] Fractal Dimension Octree Grid Generation: %1 s.").arg(static_cast<double>(distTime_ms) / 1000.0, 0, 'f', 2), ccMainAppInterface::STD_CONSOLE_MESSAGE);


			//Computing visualization structures and algorithm
			appInterface->setGlobalZoom();
			_spatialStructure->fillOctreesVector();
			distTime_ms = 0;
			countingBoxTimer.restart();
			_spatialStructure->computeCountingBox();
			distTime_ms += countingBoxTimer.restart();
			appInterface->dispToConsole(QString("[Counting Box] Fractal Dimension Differences Computation: %1 s.").arg(static_cast<double>(distTime_ms) / 1000.0, 0, 'f', 2), ccMainAppInterface::STD_CONSOLE_MESSAGE);


			//Creating final displayed cloud with its own scalar field
			ccPointCloud* resultingCloud = new ccPointCloud();
			resultingCloud->setName("Fractal Dimension Differences Cloud");
			resultingCloud->reserve(totalPoints);
			resultingCloud->reserveTheRGBTable();
			resultingCloud->toggleColors();
			resultingCloud->setVisible(true);

			ccScalarField* sf = new ccScalarField("Fractal Dimension Differences");
			int sfIdx = resultingCloud->addScalarField(sf);
			resultingCloud->setCurrentScalarField(sfIdx);
			resultingCloud->setCurrentDisplayedScalarField(sfIdx);
			resultingCloud->enableScalarField();

			ccHObject* root = appInterface->dbRootObject();
			ccHObject* pChildren = NULL;

			/** In order to not assign he maximum value
			ScalarType maxValue = 0;

			for (int i = 0; i < root->getChildrenNumber(); i++)
			{
				pChildren = root->getChild(i);
				if (pChildren && pChildren->isKindOf(CC_TYPES::POINT_CLOUD))
				{
					ccPointCloud* originalCloud = (ccPointCloud*)pChildren;

					if (originalCloud->isVisible()) {
						for (int i = 0; i < originalCloud->size(); ++i) {
							ScalarType scalarValue = originalCloud->getPointScalarValue(i);
							if (scalarValue > maxValue) {
								maxValue = scalarValue;
							}

						}

					}
				}
			}
			*/

			ScalarType maxValue = ScalarType(3);


			comparedClouds[0]->setVisible(false);
			comparedClouds[1]->setVisible(false);
			
			for (int i = 0; i < root->getChildrenNumber(); i++)
			{
				pChildren = root->getChild(i);
				if (pChildren && pChildren->isKindOf(CC_TYPES::POINT_CLOUD))
				{
					ccPointCloud* originalCloud = (ccPointCloud*)pChildren;

					if (originalCloud->isVisible()) {

						for (int i = 0; i < originalCloud->size(); ++i) {

							auto point = originalCloud->getPoint(i);
							auto color = originalCloud->getPointColor(i);

							resultingCloud->addPoint(*point);
							resultingCloud->addColor(color);

							ScalarType scalarValue = originalCloud->getPointScalarValue(i);

							if (scalarValue == DEFAULT_DIFF_VALUE) {
								scalarValue = maxValue;

							}
							resultingCloud->addPointScalarValue(scalarValue);
						}
						originalCloud->setVisible(false);
					}
				}

			}

			//Setting up the scalar field to be displayed
			sf->computeMinAndMax();
			resultingCloud->showSF(true);


			if (visualizationChecked == true) {
				_spatialStructure->computeOctreeVisualization();
				temporalSeriesPluginInterface->linkWith(appInterface->getActiveGLWindow());
				appInterface->registerOverlayDialog(temporalSeriesPluginInterface, Qt::TopLeftCorner);
				temporalSeriesPluginInterface->start();
			}

			appInterface->addToDB(resultingCloud);
			appInterface->refreshAll();

			//	REMOVING
			/**
			ccHObject::Container toRemove;
			for (int i = 0; i < root->getChildrenNumber(); i++)
			{
				pChildren = root->getChild(i);
				if (pChildren && pChildren->isKindOf(CC_TYPES::POINT_CLOUD))
				{
					ccPointCloud* originalCloud = (ccPointCloud*)pChildren;
					if (originalCloud != comparedClouds[0] && originalCloud != comparedClouds[1] && originalCloud != resultingCloud) {
						toRemove.push_back(pChildren);
					}
				}
			}

			for (auto element : toRemove) {
				appInterface->removeFromDB(element);
			}
			*/

		}
		else {

			appInterface->dispToConsole("[Temporal Series Plugin] Comparison cancelled", ccMainAppInterface::STD_CONSOLE_MESSAGE);
		}

	
		/*** HERE ENDS THE ACTION ***/
	}

	

	/**
	*	@brief Enable or disable the octrees visualization
	*/
	void octreeButtonAction(ccMainAppInterface* appInterface)
	{

		_spatialStructure->changeOctreeState();
	}

	/**
	*	@brief Enable or disable the nodes visualization
	*/
	void nodeButtonAction(ccMainAppInterface* appInterface)
	{
		_spatialStructure->changeNodeState();
	}

	/**
	*	@brief Close the UI
	*/
	void closeButtonAction(ccMainAppInterface* appInterface, TemporalSeriesPluginInterface* temporalSeriesPluginInterface)
	{
		temporalSeriesPluginInterface->stop(true);
		appInterface->unregisterOverlayDialog(temporalSeriesPluginInterface);
	}

	void computePreviewParameters(int* numberOfCells, int* minimumOctreeNodePoints, int* maximumOctreeDepthLevel,
		const std::vector<ccPointCloud*>& comparedClouds, float* cellSize, ccMainAppInterface* appInterface)
	{
		int cloudAPoints = comparedClouds[0]->size();
		int cloudBPoints = comparedClouds[1]->size();

		PointCoordinateType maxBoxDimCloudA = comparedClouds[0]->getBB_recursive().getMaxBoxDim();
		PointCoordinateType maxBoxDimCloudB = comparedClouds[1]->getBB_recursive().getMaxBoxDim();

		PointCoordinateType maxDimBothClouds = maxBoxDimCloudA > maxBoxDimCloudB ? maxBoxDimCloudA : maxBoxDimCloudB;

	
		*cellSize = (maxDimBothClouds / *numberOfCells) *1.001;	//Coordinates fitting
		//*cellSize = *numberOfCells;	//EXPERIMENTATION ONLY

	}

	void shiftCloudsOrigin(std::vector<ccPointCloud*>& comparedClouds, ccMainAppInterface* appInterface) {
		
		CCVector3 minCornerCloudA = comparedClouds[0]->getBB_recursive().minCorner();
		CCVector3 minCornerCloudB = comparedClouds[1]->getBB_recursive().minCorner();

		CCVector3 minCornerBothClouds;
		minCornerBothClouds.x = minCornerCloudA.x < minCornerCloudB.x ? minCornerCloudA.x : minCornerCloudB.x;
		minCornerBothClouds.y = minCornerCloudA.y < minCornerCloudB.y ? minCornerCloudA.y : minCornerCloudB.y;
		minCornerBothClouds.z = minCornerCloudA.z < minCornerCloudB.z ? minCornerCloudA.z : minCornerCloudB.z;


		comparedClouds[0]->translate(CCVector3(-minCornerBothClouds.x, -minCornerBothClouds.y, -minCornerBothClouds.z));
		comparedClouds[1]->translate(CCVector3(-minCornerBothClouds.x, -minCornerBothClouds.y, -minCornerBothClouds.z));

	}

}
