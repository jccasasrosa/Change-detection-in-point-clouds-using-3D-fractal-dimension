#include "OctreeGrid.h"
#include <fstream>
#include <string>
#include <qtconcurrentmap.h>
#include <qthread.h>
#include <QtConcurrent\qtconcurrentrun.h>

/**
*	@brief Default contructor
*/
OctreeGrid::OctreeGrid()
{
	_cellSize = 0;
	_octreeButtonToggled = false;
	_nodeButtonToggled = false;
	_polilynesCreated = false;
}

/**
*	@brief Parameterized constructor with the cell size
*/
OctreeGrid::OctreeGrid(float cellSize, int minimumNumberOfPointsPerNode, int maxDepthLevel)
{
	_cellSize = cellSize;
	_minimumNumberOfPointsPerNode = minimumNumberOfPointsPerNode;
	_maxDepthLevel = maxDepthLevel;
	_octreeButtonToggled = false;
	_nodeButtonToggled = false;
	_polilynesCreated = false;
}


/**
*	@brief Compute the bounding box from the full octree grid
*/
void OctreeGrid::computeBoundingBox(const std::vector<glm::vec3>& pointsPositions)
{
	_boundingBox.push_back(glm::vec3(DBL_MAX, DBL_MAX, DBL_MAX));	//Min values
	_boundingBox.push_back(glm::vec3(DBL_MIN, DBL_MIN, DBL_MIN));	//Max values

	for (glm::vec3 point : pointsPositions) {

		if (point.x < _boundingBox[0].x) {
			_boundingBox[0].x = point.x;
		}
		else if (point.x > _boundingBox[1].x) {
			_boundingBox[1].x = point.x;
		}

		if (point.y < _boundingBox[0].y) {
			_boundingBox[0].y = point.y;
		}
		else if (point.y > _boundingBox[1].y) {
			_boundingBox[1].y = point.y;
		}

		if (point.z < _boundingBox[0].z) {
			_boundingBox[0].z = point.z;
		}
		else if (point.z > _boundingBox[1].z) {
			_boundingBox[1].z = point.z;
		}

	}
}

/**
*	@brief Generation of the octree grid instance and processing of the points from each cloud
*/
void OctreeGrid::loadOctreeGrid(const std::vector<glm::vec3>& pointsPositions, const std::vector<glm::vec3>& pointsColors,
	const unsigned int& numberOfClouds, const unsigned int& subcloud)
{

	for (int i = 0; i < pointsPositions.size(); ++i) {

		const glm::vec3& pointPosition = pointsPositions[i];
		const glm::vec3& pointColor = pointsColors[i];

		glm::ivec3 key(pointPosition / _cellSize);
		if (_octrees.find(key) == _octrees.end()) {
			_octrees[key] = new Octree(new Node(NULL, _cellSize, numberOfClouds, 1, _minimumNumberOfPointsPerNode, _maxDepthLevel, key, (_cellSize/2)), _cellSize, key, _minimumNumberOfPointsPerNode, _maxDepthLevel);
		}

		_indexStructure[key].push_back(i);

	}


	std::unordered_map< glm::ivec3, std::vector<glm::vec3>, glm::ivec3Hash > newPointMap;
	std::unordered_map< glm::ivec3, std::vector<glm::vec3>, glm::ivec3Hash > newColorMap;


	for (auto values : _indexStructure) {
		for (int i = 0; i < values.second.size(); ++i) {
			newPointMap[values.first].push_back(pointsPositions[values.second[i]]);
			newColorMap[values.first].push_back(pointsColors[values.second[i]]);
		}
	}

	
	for (auto& values : _octrees) {
		values.second->setAppinterface(_appInterface);
		values.second->addPointsToCloud(newPointMap[values.first], newColorMap[values.first], subcloud);
		values.second->setRootNodeCorners();
	}

	_indexStructure.clear();
	
	
	/**
	std::unordered_map< glm::ivec3, std::vector<glm::vec3>, glm::ivec3Hash > newPointMap, newColorMap;
	glm::ivec3 key;
	// Repartir puntos por celdas
	for (int i = 0; i < pointsPositions.size(); ++i) {
		glm::vec3 pointPosition = pointsPositions[i];
		glm::vec3 pointColor = pointsColors[i];

		key = pointPosition / _cellSize;

		newPointMap[key].push_back(pointPosition);
		newColorMap[key].push_back(pointColor);
	}

	// Construir octree por celda
	for (auto& points : newPointMap) {
		key = points.first;

		Octree* octree = new Octree(new Node(NULL, _cellSize, numberOfClouds, 1, _minimumNumberOfPointsPerNode, _maxDepthLevel, key, (_cellSize / 2)), _cellSize, key, _minimumNumberOfPointsPerNode, _maxDepthLevel);
		octree->setAppinterface(_appInterface);
		octree->addPointsToCloud(newPointMap[key], newColorMap[key], subcloud);
		octree->setRootNodeCorners();

		_octrees[key] = octree;
	}
	*/
	
	
}

/**
*	@brief Get the bounding box of the octree grid
*/
const std::vector<glm::dvec3>& OctreeGrid::getBoundingBoxVector()
{
	return _boundingBox;
}
 
/**
*	@brief Get the data structure that contains all the octrees of the grid
*/
const std::unordered_map<glm::ivec3, Octree*, glm::fvec3Hash>& OctreeGrid::getOctreeGrid()
{
	return _octrees;
}

/**
*	@brief Complete the subdivision of all the octrees of the grid
*/
void OctreeGrid::completeSubdivide()
{
	for (auto& it : _octrees) {
		it.second->completeSubdivide();
	}
}

/**
*	@brief Get the visualization state of the octrees
*/
const bool& OctreeGrid::getOctreeButtonState()
{
	return _octreeButtonToggled;
}

/**
*	@brief Get the visualization state of the nodes
*/
const bool& OctreeGrid::getNodeButtonState()
{
	return _nodeButtonToggled;
}

/**
*	@brief Display or disable the octree visualization
*/
void OctreeGrid::changeOctreeState()
{
	_octreeButtonToggled = !_octreeButtonToggled;
	changeOctreeVisibility(_octreeButtonToggled);
}

/**
*	@brief Display or disable the node visualization
*/
void OctreeGrid::changeNodeState()
{
	_nodeButtonToggled = !_nodeButtonToggled;
	changeNodesVisibility(_nodeButtonToggled);
}

/**
*	@brief Set the app interface object to the octree grid instance
*/
void OctreeGrid::setAppinterface(ccMainAppInterface* appInterface)
{
	_appInterface = appInterface;
	
}

/**
*	@brief Aux function to compute the octrees and nodes polylines
*/
void OctreeGrid::auxComputeVisualization(std::vector<ccPointCloud*>* pointCloudVector, std::vector<ccPolyline*>* polilyneVector, const std::vector<glm::dvec3>& boundingBoxVector, const ccColor::Rgba& color)
{


	ccPointCloud* auxCloud = new ccPointCloud();
	auxCloud->addPoint(CCVector3(boundingBoxVector[0].x, boundingBoxVector[0].y, boundingBoxVector[0].z));
	auxCloud->addPoint(CCVector3(boundingBoxVector[1].x, boundingBoxVector[0].y, boundingBoxVector[0].z));
	auxCloud->addPoint(CCVector3(boundingBoxVector[1].x, boundingBoxVector[1].y, boundingBoxVector[0].z));
	auxCloud->addPoint(CCVector3(boundingBoxVector[0].x, boundingBoxVector[1].y, boundingBoxVector[0].z));
	auxCloud->addPoint(CCVector3(boundingBoxVector[0].x, boundingBoxVector[0].y, boundingBoxVector[0].z));

	auxCloud->addPoint(CCVector3(boundingBoxVector[0].x, boundingBoxVector[0].y, boundingBoxVector[1].z));
	auxCloud->addPoint(CCVector3(boundingBoxVector[1].x, boundingBoxVector[0].y, boundingBoxVector[1].z));
	auxCloud->addPoint(CCVector3(boundingBoxVector[1].x, boundingBoxVector[0].y, boundingBoxVector[0].z));
	auxCloud->addPoint(CCVector3(boundingBoxVector[1].x, boundingBoxVector[0].y, boundingBoxVector[1].z));
	auxCloud->addPoint(CCVector3(boundingBoxVector[1].x, boundingBoxVector[1].y, boundingBoxVector[1].z));
	auxCloud->addPoint(CCVector3(boundingBoxVector[1].x, boundingBoxVector[1].y, boundingBoxVector[0].z));
	auxCloud->addPoint(CCVector3(boundingBoxVector[1].x, boundingBoxVector[1].y, boundingBoxVector[1].z));
	auxCloud->addPoint(CCVector3(boundingBoxVector[0].x, boundingBoxVector[1].y, boundingBoxVector[1].z));
	auxCloud->addPoint(CCVector3(boundingBoxVector[0].x, boundingBoxVector[1].y, boundingBoxVector[0].z));
	auxCloud->addPoint(CCVector3(boundingBoxVector[0].x, boundingBoxVector[1].y, boundingBoxVector[1].z));
	auxCloud->addPoint(CCVector3(boundingBoxVector[0].x, boundingBoxVector[0].y, boundingBoxVector[1].z));
	pointCloudVector->push_back(auxCloud);
	ccPolyline* polyline = new ccPolyline(auxCloud);
	polyline->addPointIndex(0, 16);
	polyline->setColor(color);
	polyline->toggleColors();
	polyline->setWidth(5);
	polyline->setVisible(false);
	polilyneVector->push_back(polyline);
	_appInterface->addToDB(polyline);
}


/**
*	@brief Function to compute the octrees visualization tools
*/
void OctreeGrid::computeOctreeVisualization()
{
	for (auto octree : _octrees) {
		auxComputeVisualization(&_octreesAuxCloud, &_octreesPolilynes, octree.second->getBoundingBoxVector(), ccColor::Rgba(0, 200, 0, 1));

		Node* rootNode = octree.second->getRootNode();
		computeNodeVisualization(rootNode);	
	}
}

/**
*	@brief Function to compute the node visualization tools
*/
void OctreeGrid::computeNodeVisualization(Node * node)
{

	std::vector<glm::dvec3> boundingBoxCorner;
	glm::dvec3* minCorner = node->getMinCorner();
	glm::dvec3* maxCorner = node->getMaxCorner();
	boundingBoxCorner.push_back(*minCorner);
	boundingBoxCorner.push_back(*maxCorner);

	auxComputeVisualization(&_nodesAuxCloud, &_nodesPolilynes, boundingBoxCorner, ccColor::Rgba(0, 0, 200, 1));	//BLUE


	std::vector<Node*> childrenNodes;
	childrenNodes = *node->getChildrens();

	if (childrenNodes.size() > 0) {
		for (int i = 0; i < childrenNodes.size(); ++i) {
			computeChildVisualization(childrenNodes[i]);
			
		}
	}
}

void OctreeGrid::computeChildVisualization(Node* node)
{
	std::vector<glm::dvec3> boundingBoxCornerChild;
	glm::dvec3* minCornerChild = node->getMinCorner();
	glm::dvec3* maxCornerChild = node->getMaxCorner();
	boundingBoxCornerChild.push_back(*minCornerChild);
	boundingBoxCornerChild.push_back(*maxCornerChild);

	if (node->getLevel() == 2) {
		auxComputeVisualization(&_nodesAuxCloud, &_nodesPolilynes, boundingBoxCornerChild, ccColor::Rgba(244, 194, 194, 1));	//PINK
	}
	else if (node->getLevel() == 3) {
		auxComputeVisualization(&_nodesAuxCloud, &_nodesPolilynes, boundingBoxCornerChild, ccColor::Rgba(250, 0, 0, 1));	//RED
	}
	else {
		auxComputeVisualization(&_nodesAuxCloud, &_nodesPolilynes, boundingBoxCornerChild, ccColor::Rgba(0, 255, 0, 1));	//GREEN
	}

	std::vector<Node*> childrenNodes;
	childrenNodes = *node->getChildrens();

	if (childrenNodes.size() > 0) {
		for (int i = 0; i < childrenNodes.size(); ++i) {
			computeChildVisualization(childrenNodes[i]);
		}
	}
}

/**
*	@brief Function to fill the vector which contains the octrees from the octree grid in order to parallelize in CPU the counting box
*	algorithm in each octree
*/
void OctreeGrid::fillOctreesVector()
{
	for (auto& values : _octrees) {
		_octreesForParallelization.push_back(values.second);
	}
}

/**
*	@brief Compute the counting box algorithm for an octree
*/

void callCountingBox(Octree* octree) {
	octree->computeCountingBox();
}

/**
*	@brief Compute the counting box algorithm for each octree using CPU threading parallelization
*/
void OctreeGrid::computeCountingBox()
{
	//Parallelization of the counting box per octree
	int maxThreadCount = QThread::idealThreadCount();
	QThreadPool::globalInstance()->setMaxThreadCount(maxThreadCount);
	QtConcurrent::blockingMap(_octreesForParallelization, callCountingBox);


}

/**
*	@brief Enable or disable the octrees visualization
*/
void OctreeGrid::changeOctreeVisibility(const bool& state)
{
	for (int i = 0; i < _octreesPolilynes.size(); ++i) {
		_octreesPolilynes[i]->setVisible(state);
	}
	_appInterface->redrawAll();
}

/**
*	@brief Enable or disable the nodes visualization
*/
void OctreeGrid::changeNodesVisibility(const bool& state)
{
	for (int i = 0; i < _nodesPolilynes.size(); ++i) {
		_nodesPolilynes[i]->setVisible(state);
	}
	_appInterface->redrawAll();
}

