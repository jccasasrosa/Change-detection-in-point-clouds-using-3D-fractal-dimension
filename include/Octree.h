#pragma once
#include "Node.h"

class Octree {
private:
	Node* _rootNode;	//Pointer to the root node
	glm::ivec3 _coordinates;	//Coordinates of the octree in the space
	float _cellSize;	//Size of each cell
	int _minimumNumberOfPointsPerNode;	//Minimum number of points for each node
	int _maxDepthLevel;	//Maximum octree depth level
	std::vector<glm::dvec3> _boundingBox;	//Bounding box of the octree
	ccMainAppInterface* _appInterface;

public:
	Octree();
	Octree(Node* rootNode, float cellSize, const glm::dvec3& coordinates, int minimumNumberOfPointsPerNode, int maxDepthLevel);	//Parameterized constructor

	void addPointsToCloud(const std::vector<glm::vec3>& pointPosition, const std::vector<glm::vec3>& pointColor, const unsigned int& subcloud);
	void completeSubdivide();

	const std::vector<glm::dvec3>& getBoundingBoxVector();
	Node* getRootNode();

	void computeCountingBox();
	double getDifferenceBetweenClouds();

	void setAppinterface(ccMainAppInterface* appInterface);
	bool areCloudsComparable();

	void setRootNodeCorners();

};