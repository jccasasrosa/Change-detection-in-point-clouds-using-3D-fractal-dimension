#include "Octree.h"
#include <fstream>

/**
*	@brief Default constructor
*/
Octree::Octree()
{
	_rootNode = NULL;
}


/**
*	@brief Parameterized constructor
*/
Octree::Octree(Node* rootNode, float cellSize, const glm::dvec3& coordinates, int minimumNumberOfPointsPerNode, int maxDepthLevel)
{
	_rootNode = rootNode;
	_coordinates = coordinates;
	_cellSize = cellSize;
	_minimumNumberOfPointsPerNode = minimumNumberOfPointsPerNode;
	_maxDepthLevel = maxDepthLevel;
	_boundingBox.push_back(glm::dvec3(coordinates.x * _cellSize, coordinates.y * _cellSize, coordinates.z * _cellSize));
	_boundingBox.push_back(glm::dvec3((coordinates.x *_cellSize) + _cellSize, (coordinates.y*_cellSize) + _cellSize, (coordinates.z * _cellSize) + _cellSize));
}

/**
*	@brief Fill the root node of the octree
*/
void Octree::addPointsToCloud(const std::vector<glm::vec3>& pointPosition, const std::vector<glm::vec3>& pointColor, const unsigned int& subcloud)
{
	_rootNode->addPointsToCloud(pointPosition, pointColor, subcloud);
}

/**
*	@brief Complete the subdivision of the octree
*/
void Octree::completeSubdivide()
{
	_rootNode->computeBoundingBox();
	_rootNode->completeSubdivide();
}

/**
*	@brief Get he bounding box of each octree
*/
const std::vector<glm::dvec3>& Octree::getBoundingBoxVector()
{
	return _boundingBox;
}

/**
*	@brief Get the root node
*/
Node* Octree::getRootNode()
{
	return _rootNode;
}


/**
*	@brief Apply the counting box algorithm to the root node
*/
void Octree::computeCountingBox()
{
	_rootNode->assignCountingBox(_rootNode);
}
/**
*	@brief Get the difference between the fractal dimension of different point clouds
*/
double Octree::getDifferenceBetweenClouds()
{
	return _rootNode->getDifferenceBetweenClouds();
}

/**
*	@brief Set the app interface object to the octree and root node instance
*/
void Octree::setAppinterface(ccMainAppInterface* appInterface)
{
	_appInterface = appInterface;
	_rootNode->setAppinterface(_appInterface);
}

/**
*	@brief Get if both cloud of the root node are comparable
*/
bool Octree::areCloudsComparable()
{
	return _rootNode->areCloudsComparable();
}

void Octree::setRootNodeCorners()
{
	_rootNode->computeBoundingBox();
	std::vector<glm::dvec3> bboxRootNode = _rootNode->getBoundingBoxVector();
	_rootNode->setCorners(bboxRootNode[0], bboxRootNode[1]);
}


