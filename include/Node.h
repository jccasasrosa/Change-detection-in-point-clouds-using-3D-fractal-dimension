#pragma once

#include <vector>
#include <unordered_set>
#include <glm.hpp>
#include <ccPointCloud.h>
#include <ccMainAppInterface.h>
#define MAX_NUMBER_OF_POINTS_PER_NODE 5000
#define MIN_NUMBER_OF_POINTS_PER_NODE 1000
#define NUMBER_OF_SUBDIVISIONS 2
#define MAX_BOX_COUNTING_RESOLUTION 10
#define DEFAULT_DIFF_VALUE -99
#define LEVEL 2



class Node {

private:
	Node* _parent;	//Pointer to the parent node
	std::vector<Node*> _childrens;	//Pointers to each of its 8 children
	std::vector<glm::dvec3> _boundingBox;	//Bounding box of the node
	float _cellSize;	//Size of the cell

	std::vector<ccPointCloud*> _clouds;	//Structure that contains positions and colors
	ccMainAppInterface* _appInterface;

	unsigned int _numberOfSubclouds;
	long _numberOfPoints;
	int _level;


	std::vector<std::vector<unsigned int> > _boxCounting;	//Vector vector containing the box count for each cloud in each resolution

	double _fractalDim0;
	double _fractalDim1;
	double _differenceBetweenClouds;	//Concrete differences between the slopes of each cloud

	bool _areComparable;	//Boolean indicanting if the two clouds are comparable by having enough points;

	std::vector<std::vector<ccColor::Rgb>> _originalColors;	//To paint again the original color after painting differences


public:

	Node();	//Default constructor
	Node(Node* parent, float cellSize, const unsigned int& numberOfSubclouds, int level);	//Parameterized constructor
	void computeBoundingBox();
	void addPointsToCloud(const std::vector<glm::vec3>& pointPosition, const std::vector<glm::vec3>& pointColor, const unsigned int& subcloud);
	int computeChildrenIndex(const glm::vec3& pointPosition);
	void completeSubdivide();
	const std::vector<Node*>& getChildrens();
	const std::vector<glm::dvec3>& getBoundingBoxVector();
	void setAppinterface(ccMainAppInterface* appInterface);
	std::vector<ccPointCloud*> * getClouds();
	void computeCountingBox();
	void computeCountingBoxChildren();
	const std::vector<double>& computeLinearRegression(const std::vector<double>& x, const std::vector<double>& y);
	double getSlope(const std::vector<double>& x, const std::vector<double>& y);
	double getDifferenceBetweenClouds();
	bool areCloudsComparable();
	int getLevel();
	void storePoints(int index);
};