#pragma once

#include <vector>
#include <unordered_set>
#include <glm.hpp>
#include <ccPointCloud.h>
#include <ccMainAppInterface.h>
#define NUMBER_OF_SUBDIVISIONS 2
#define MAX_BOX_COUNTING_RESOLUTION 10
#define DEFAULT_DIFF_VALUE -99
#define LEVEL 2
typedef std::pair<glm::dvec3, glm::dvec3> Box;



class Node {

private:
	Node* _parent;	//Pointer to the parent node
	std::vector<Node*> _childrens;	//Pointers to each of its 8 children
	std::vector<glm::dvec3> _boundingBox;	//Bounding box of the cloud of the node
	float _cellSize;	//Size of the cell
	glm::ivec3 _coordinates;	//Coordinates of the node in the space

	glm::dvec3 _minCorner;	//Lower corner of the node
	glm::dvec3 _maxCorner;	//Upper corner of the node

	std::vector<ccPointCloud*> _clouds;	//Structure that contains positions and colors
	ccMainAppInterface* _appInterface;	//Interface of the CloudCompare application

	unsigned int _numberOfSubclouds;
	long _numberOfPoints;
	int _level;
	int _minimumNumberOfPointsPerNode;
	int _maxDepthLevel;

	std::vector<std::vector<unsigned int> > _boxCounting;	//Vector vector containing the box count for each cloud in each resolution
	std::vector<double> _squareDimVectorInverseLogarithm;	//Vector contaning the sizes of the boxes

	double _squareDim;
	double _fractalDim0;
	double _fractalDim1;
	double _differenceBetweenClouds;	//Concrete differences between the slopes of each cloud

	bool _areComparable;	//Boolean indicanting if the two clouds are comparable by having enough points;

	std::vector<std::vector<ccColor::Rgb>> _originalColors;	//To paint again the original color after painting differences


public:

	Node();	//Default constructor
	Node(Node* parent, float cellSize, const unsigned int& numberOfSubclouds, int level, int minimumNumberOfPointsPerNode, int maxDepthLevel, glm::ivec3 coordinates, double squareDim);	//Parameterized constructor
	void computeBoundingBox();
	void addPointsToCloud(const std::vector<glm::vec3>& pointPosition, const std::vector<glm::vec3>& pointColor, const unsigned int& subcloud);
	int computeChildrenIndex(const glm::vec3& pointPosition);
	void setCorners(const glm::dvec3& minCorner, const glm::dvec3& maxCorner);
	Box computeCorners(int childrenIndex);
	glm::dvec3* getMinCorner();
	glm::dvec3* getMaxCorner();
	void completeSubdivide();
	std::vector<Node*>* getChildrens();
	const std::vector<glm::dvec3>& getBoundingBoxVector();
	void setAppinterface(ccMainAppInterface* appInterface);
	std::vector<ccPointCloud*> * getClouds();
	void computeCountingBox(Node* node);
	void assignCountingBox(Node* node);
	const std::vector<double>& computeLinearRegression(const std::vector<double>& x, const std::vector<double>& y);
	double getSlope(const std::vector<double>& x, const std::vector<double>& y);
	double getDifferenceBetweenClouds();
	bool areCloudsComparable();
	int getLevel();
	unsigned int getNumberOfSubclouds();
	const std::vector<std::vector<unsigned int> >& getBoxCounting();
	const std::vector<double>& getSquareDimVectorInverseLogarithm();
	const double& getSquareDim();
	void setFD0(double FD);
	void setFD1(double FD);
	void setDiff(double diff);
	void setComparable(bool comnparable);
	void setBoxCountingValue(int cloud, int position, unsigned int value);
	void setSquareDimVectorInverseLogarithmValue(int position, double value);
	const unsigned int& getBoxCountingValue(Node* node, int cloud, int position);
	const double& getSquareDimVectorInverseLogarithmValue(Node* node, int position);
};