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

	void performActionA( ccMainAppInterface *appInterface, TemporalSeriesPluginInterface* temporalSeriesPluginInterface)
	{
		if ( appInterface == nullptr )
		{
			// The application interface should have already been initialized when the plugin is loaded
			Q_ASSERT( false );
			
			return;
		}
		
	
		//File dialog to select files
		QFileDialog myDialog;
		myDialog.setWindowTitle("Select the files of the temporal series");
		//myDialog.setDirectory("C:\\Users\\UJA\\Desktop\\Experimentacion");
		myDialog.setFileMode(QFileDialog::ExistingFiles);
		myDialog.setNameFilters(QStringList() << "*.laz *.las");
		myDialog.exec();

		//Obtention of the files
		QStringList files = myDialog.selectedFiles();
		unsigned int numberOfFiles = files.size();
		QString message;

		//Creation of the reader object
		LASreaderLAS reader;

		//Creation of the spatial structure
		_spatialStructure = new OctreeGrid(CELL_SIZE);
		_spatialStructure->setAppinterface(appInterface);

		//Definition of points of each file
		long totalPoints = 0;

		//Structure for retrieving times
		QElapsedTimer countingBoxTimer;
		qint64 distTime_ms = 0;

		//Iterating through the files and generating the octree
		for (int i = 0; i < numberOfFiles; ++i) {

			std::vector<glm::vec3> pointsPosition;
			std::vector<glm::vec3> pointsColor;

			QString file = files[i];

			reader.open(file.toStdString().c_str());
			long numberOfPoints = reader.header.number_of_point_records;
			totalPoints += numberOfPoints;
			message = QString::number(numberOfPoints);
			appInterface->dispToConsole("Number of points records: " + message, ccMainAppInterface::STD_CONSOLE_MESSAGE);

			LASpoint& point = reader.point;
			unsigned int count = 0;

			//Simple window to show the progress of the reading process
			QProgressDialog progressDialog(file, NULL, 0, numberOfPoints);
			progressDialog.setWindowTitle("Loading point cloud ...");
			progressDialog.setMinimumWidth(500);
			progressDialog.setMinimumHeight(200);
			progressDialog.setMinimumDuration(0);


			while (reader.read_point()) {

				LASpoint& point = reader.point;
				float coordX = point.get_x();
				float coordY = point.get_y();
				float coordZ = point.get_z();
				CCVector3 vector(coordX, coordY, coordZ);

				int colorR = point.get_R() / 256;
				int colorG = point.get_G() / 256;
				int colorB = point.get_B() / 256;
				ccColor::Rgb color(colorR, colorG, colorB);


				pointsPosition.push_back(glm::vec3(coordX, coordY, coordZ));
				pointsColor.push_back(glm::vec3(colorR, colorG, colorB));

				++count;
				if (count % LOAD_INTERVAL == 0) {
					progressDialog.setValue(count);
					QApplication::processEvents();
				}
			}

			progressDialog.close();

			appInterface->dispToConsole("Computing bounding box of the Octree Grid", ccMainAppInterface::STD_CONSOLE_MESSAGE);
			appInterface->dispToConsole("Loading octree grid", ccMainAppInterface::STD_CONSOLE_MESSAGE);

			countingBoxTimer.start();
			QString numberOfNodes = QString(_spatialStructure->loadOctreeGrid(pointsPosition, pointsColor, numberOfFiles, i).c_str());
			distTime_ms += countingBoxTimer.restart();
			appInterface->dispToConsole(numberOfNodes, ccMainAppInterface::STD_CONSOLE_MESSAGE);	
		}
		
		//Finished reading, completing subdivision of each octree in the octree grid
		countingBoxTimer.restart();
		_spatialStructure->completeSubdivide();
		distTime_ms += countingBoxTimer.restart();
		appInterface->dispToConsole(QString("[Counting Box] Fractal Dimension Octree Grid Generation: %1 s.").arg(static_cast<double>(distTime_ms) / 1000.0, 0, 'f', 3), ccMainAppInterface::STD_CONSOLE_MESSAGE);

		//Computing visualization structures and algorithm
		appInterface->setGlobalZoom();
		_spatialStructure->computeOctreeVisualization();
		_spatialStructure->fillOctreesVector();
		distTime_ms = 0;
		countingBoxTimer.restart();
		_spatialStructure->computeCountingBox();
		distTime_ms += countingBoxTimer.restart();
		_spatialStructure->storeOctrees();

		appInterface->dispToConsole(QString("[Counting Box] Fractal Dimension Differences Computation: %1 s.").arg(static_cast<double>(distTime_ms) / 1000.0, 0, 'f', 3), ccMainAppInterface::STD_CONSOLE_MESSAGE);

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
		
		maxValue = ScalarType(20);	//Asignación mayor valor posible

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

							auto scalarValue = originalCloud->getPointScalarValue(i);
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
		appInterface->addToDB(resultingCloud);
		appInterface->refreshAll();
		
		temporalSeriesPluginInterface->linkWith(appInterface->getActiveGLWindow());
		appInterface->registerOverlayDialog(temporalSeriesPluginInterface, Qt::TopLeftCorner);
		temporalSeriesPluginInterface->start();

	
		/*** HERE ENDS THE ACTION ***/
	}

	/**
	*	@brief Method for creating the bounding box of the octrees and the nodes
	*/
	void drawBoundingBox(const std::vector<glm::vec3>& boundingBox, ccMainAppInterface* appInterface, const ccColor::Rgba& color, const bool& enable)
	{
		std::vector<glm::vec3> octreeGridBoundingBox = boundingBox;


		ccPointCloud* auxCloud = new ccPointCloud();
		auxCloud->addPoint(CCVector3(octreeGridBoundingBox[0].x, octreeGridBoundingBox[0].y, octreeGridBoundingBox[0].z));
		auxCloud->addPoint(CCVector3(octreeGridBoundingBox[1].x, octreeGridBoundingBox[0].y, octreeGridBoundingBox[0].z));
		auxCloud->addPoint(CCVector3(octreeGridBoundingBox[1].x, octreeGridBoundingBox[1].y, octreeGridBoundingBox[0].z));
		auxCloud->addPoint(CCVector3(octreeGridBoundingBox[0].x, octreeGridBoundingBox[1].y, octreeGridBoundingBox[0].z));
		auxCloud->addPoint(CCVector3(octreeGridBoundingBox[0].x, octreeGridBoundingBox[0].y, octreeGridBoundingBox[0].z));

		auxCloud->addPoint(CCVector3(octreeGridBoundingBox[0].x, octreeGridBoundingBox[0].y, octreeGridBoundingBox[1].z));
		auxCloud->addPoint(CCVector3(octreeGridBoundingBox[1].x, octreeGridBoundingBox[0].y, octreeGridBoundingBox[1].z));
		auxCloud->addPoint(CCVector3(octreeGridBoundingBox[1].x, octreeGridBoundingBox[0].y, octreeGridBoundingBox[0].z));
		auxCloud->addPoint(CCVector3(octreeGridBoundingBox[1].x, octreeGridBoundingBox[0].y, octreeGridBoundingBox[1].z));
		auxCloud->addPoint(CCVector3(octreeGridBoundingBox[1].x, octreeGridBoundingBox[1].y, octreeGridBoundingBox[1].z));
		auxCloud->addPoint(CCVector3(octreeGridBoundingBox[1].x, octreeGridBoundingBox[1].y, octreeGridBoundingBox[0].z));
		auxCloud->addPoint(CCVector3(octreeGridBoundingBox[1].x, octreeGridBoundingBox[1].y, octreeGridBoundingBox[1].z));
		auxCloud->addPoint(CCVector3(octreeGridBoundingBox[0].x, octreeGridBoundingBox[1].y, octreeGridBoundingBox[1].z));
		auxCloud->addPoint(CCVector3(octreeGridBoundingBox[0].x, octreeGridBoundingBox[1].y, octreeGridBoundingBox[0].z));
		auxCloud->addPoint(CCVector3(octreeGridBoundingBox[0].x, octreeGridBoundingBox[1].y, octreeGridBoundingBox[1].z));
		auxCloud->addPoint(CCVector3(octreeGridBoundingBox[0].x, octreeGridBoundingBox[0].y, octreeGridBoundingBox[1].z));
		ccPolyline* polyline = new ccPolyline(auxCloud);
		polyline->addPointIndex(0, 16);
		polyline->setColor(color);
		polyline->toggleColors();
		polyline->setWidth(5);
		polyline->setVisible(false);
		appInterface->addToDB(polyline);
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
}
