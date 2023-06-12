#include "Node.h"
#include <fstream>
#include <math.h>
#include <algorithm>
#include <numeric>
#include <ccScalarField.h>
#include <DistanceComputationTools.h>
#include <laswriter_las.hpp>
#include <experimental/filesystem>
namespace fs = std::experimental::filesystem;

/**
*	@brief Default constructor
*/
Node::Node() {
	_parent = NULL;
}

/**
*	@brief Parameterized constructor
*/
Node::Node(Node* parent, float cellSize, const unsigned int& numberOfSubclouds, int level, 
	int minimumNumberOfPointsPerNode, int maxDepthLevel, glm::ivec3 coordinates, double squareDim)
{
	_parent = parent;
	_cellSize = cellSize;
	_numberOfSubclouds = numberOfSubclouds;
	_level = level;
	_minimumNumberOfPointsPerNode = minimumNumberOfPointsPerNode;
	_maxDepthLevel = maxDepthLevel;
	_coordinates = coordinates;
	_squareDim = squareDim;
	//_minCorner = minCorner;
	//_maxCorner = maxCorner;

	for (int i = 0; i < numberOfSubclouds; ++i) {
		_clouds.push_back(new ccPointCloud());
	}

	_numberOfPoints = 0;
	_appInterface = 0;

	_boxCounting.resize(numberOfSubclouds);
	for (int i = 0; i < numberOfSubclouds; ++i) {
		_boxCounting[i].resize(MAX_BOX_COUNTING_RESOLUTION);
	}
	for (int i = 0; i < numberOfSubclouds; ++i) {
		for (int j = 0; j < MAX_BOX_COUNTING_RESOLUTION; ++j) {
			_boxCounting[i][j] = 0;
		}
	}
	_squareDimVectorInverseLogarithm.resize(MAX_BOX_COUNTING_RESOLUTION);
	for (int i = 0; i < MAX_BOX_COUNTING_RESOLUTION; ++i) {
		_squareDimVectorInverseLogarithm[i] = 0;
	}
	_fractalDim0 = 0;
	_fractalDim1 = 0;
	_differenceBetweenClouds = DEFAULT_DIFF_VALUE;
	_areComparable = false;
	_originalColors.resize(_numberOfSubclouds);
}

/**
*	@brief Function to compte the bounding box from the set of points of the node
*/
void Node::computeBoundingBox()
{
	
	_boundingBox.push_back(glm::fvec3(MAXINT32, MAXINT32, MAXINT32));	//Min values
	_boundingBox.push_back(glm::fvec3(MININT32, MININT32, MININT32));	//Max values

	_numberOfPoints = _clouds[0]->size() + _clouds[1]->size();
	
	for (int i = 0; i < _clouds.size(); ++i) {
		for (int j = 0; j < _clouds[i]->size(); ++j) {

			auto point = _clouds[i]->getPoint(j);

			if (point->x < _boundingBox[0].x) {
				_boundingBox[0].x = point->x;
			}

			if (point->x > _boundingBox[1].x) {
				_boundingBox[1].x = point->x;
			}

			if (point->y < _boundingBox[0].y) {
				_boundingBox[0].y = point->y;
			}
			if (point->y > _boundingBox[1].y) {
				_boundingBox[1].y = point->y;
			}

			if (point->z < _boundingBox[0].z) {
				_boundingBox[0].z = point->z;
			}
			if (point->z > _boundingBox[1].z) {
				_boundingBox[1].z = point->z;
			}
		}
	}
	

	


	
}

/**
*	@brief Function to add points to a certain subcloud (0, 1) inside the node
*/
void Node::addPointsToCloud(const std::vector<glm::vec3>& pointPosition, const std::vector<glm::vec3>& pointColor, const unsigned int& subcloud)
{
	if (pointPosition.size() > 0) {

		_clouds[subcloud]->reserve(pointPosition.size());
		_clouds[subcloud]->reserveTheRGBTable();
		_clouds[subcloud]->toggleColors();
		_clouds[subcloud]->setVisible(true);
		QString name = "[" + QString::number(_coordinates.x) + "-" + QString::number(_coordinates.y) + "-" + QString::number(_coordinates.z) + "] level " + QString::number(_level);
		_clouds[subcloud]->setName(name);
		_numberOfPoints = pointPosition.size();

		for (int i = 0; i < pointPosition.size(); ++i) {
			CCVector3 vector(pointPosition[i].x, pointPosition[i].y, pointPosition[i].z);
			ccColor::Rgb color(pointColor[i].x, pointColor[i].y, pointColor[i].z);

			_clouds[subcloud]->addPoint(vector);
			_clouds[subcloud]->addColor(color);
			_originalColors[subcloud].push_back(color);
		}

	
			_appInterface->addToDB(_clouds[subcloud]);

	}
}

/**
*	@brief Function to obtain the octant where the point is alocated inside the node in order to subdivide
*/
int Node::computeChildrenIndex(const glm::vec3& pointPosition)
{
	int index = 0;
	//glm::dvec3 center = glm::vec3((_boundingBox[0].x + _boundingBox[1].x) / 2, (_boundingBox[0].y + _boundingBox[1].y) / 2, (_boundingBox[0].z + _boundingBox[1].z) / 2);
	//glm::dvec3 center = glm::dvec3((_minCorner.x + _maxCorner.x) / 2, (_minCorner.y + _maxCorner.y) / 2, (_minCorner.z + _maxCorner.z) / 2);
	glm::dvec3 center = (_minCorner + _maxCorner) * 0.5;

	if (pointPosition.x > center.x) {
		if (pointPosition.y > center.y) {
			if (pointPosition.z > center.z) {
				index = 0;
			}
			else {
				index = 1;
			}
		}
		else {
			if (pointPosition.z > center.z) {
				index = 2;
			}
			else {
				index = 3;
			}
		}
	}
	else {
		if (pointPosition.y > center.y) {
			if (pointPosition.z > center.z) {
				index = 4;
			}
			else {
				index = 5;
			}
		}
		else {
			if (pointPosition.z > center.z) {
				index = 6;
			}
			else {
				index = 7;
			}
		}
	}

	//glm::dvec3 childrenBoxSize = 0.5 * (_maxCorner - _minCorner);
	//glm::ivec3 octant = (glm::ivec3)(((glm::dvec3)pointPosition - _minCorner) / childrenBoxSize);

	//return (octant.z << 2) | (octant.y << 1) | octant.x;


	return index;

}

void Node::setCorners(const glm::dvec3& minCorner, const glm::dvec3& maxCorner)
{
	_minCorner = minCorner;
	_maxCorner = maxCorner;
}

Box Node::computeCorners(int childrenIndex)
{
	Box corners;


	switch (childrenIndex)
	{
	case 0:
		corners.first = glm::dvec3((_minCorner.x + _maxCorner.x) / 2, (_minCorner.y + _maxCorner.y) / 2, (_minCorner.z + _maxCorner.z) / 2);
		corners.second = glm::dvec3(_maxCorner.x,  _maxCorner.y, _maxCorner.z);
		break;
	case 1:
		corners.first = glm::dvec3((_minCorner.x + _maxCorner.x) / 2, (_minCorner.y + _maxCorner.y) / 2, _minCorner.z);
		corners.second = glm::dvec3(_maxCorner.x, _maxCorner.y, (_minCorner.z + _maxCorner.z) / 2);
		break;
	case 2:
		corners.first = glm::dvec3((_minCorner.x + _maxCorner.x) / 2, _minCorner.y, (_minCorner.z + _maxCorner.z) / 2);
		corners.second = glm::dvec3(_maxCorner.x, (_minCorner.y + _maxCorner.y) / 2, _maxCorner.z);
		break;
	case 3:
		corners.first = glm::dvec3((_minCorner.x + _maxCorner.x) / 2, _minCorner.y, _minCorner.z);
		corners.second = glm::dvec3(_maxCorner.x, (_minCorner.y + _maxCorner.y) / 2, (_minCorner.z + _maxCorner.z) / 2);
		break;
	case 4:
		corners.first = glm::dvec3(_minCorner.x, (_minCorner.y + _maxCorner.y) / 2, (_minCorner.z + _maxCorner.z) / 2);
		corners.second = glm::dvec3((_minCorner.x + _maxCorner.x) / 2, _maxCorner.y, _maxCorner.z);
		break;
	case 5:
		corners.first = glm::dvec3(_minCorner.x, (_minCorner.y + _maxCorner.y) / 2, _minCorner.z);
		corners.second = glm::dvec3((_minCorner.x + _maxCorner.x) / 2, _maxCorner.y, (_minCorner.z + _maxCorner.z) / 2);
		break;
	case 6:
		corners.first = glm::dvec3(_minCorner.x, _minCorner.y, (_minCorner.z + _maxCorner.z) / 2);
		corners.second = glm::dvec3((_minCorner.x + _maxCorner.x) / 2, (_minCorner.y + _maxCorner.y) / 2, _maxCorner.z);
		break;
	case 7:
		corners.first = glm::dvec3(_minCorner.x, _minCorner.y, _minCorner.z);
		corners.second = glm::dvec3((_minCorner.x + _maxCorner.x) / 2, (_minCorner.y + _maxCorner.y) / 2, (_minCorner.z + _maxCorner.z) / 2);
		break;
	default:
		break;
	}

	//glm::dvec3 childrenBoxSize = 0.5 * (_maxCorner - _minCorner);
	//glm::vec3 octant(childrenIndex >> 2, (childrenIndex >> 1) & 1, childrenIndex & 1);

	//Box cornersX;
	//cornersX.first = _minCorner + octant * childrenBoxSize;
	//cornersX.second = cornersX.first + childrenBoxSize;

	return corners;
}



glm::dvec3* Node::getMinCorner()
{
	return &_minCorner;
}

glm::dvec3* Node::getMaxCorner()
{
	return &_maxCorner;
}



/**
*	@brief Subdivide once both clouds are loaded
*/
void Node::completeSubdivide()
{
	unsigned int origCloudAsize = _clouds[0]->size();
	unsigned int origCloudBsize = _clouds[1]->size();
	if ((origCloudAsize > _minimumNumberOfPointsPerNode) && (origCloudBsize > _minimumNumberOfPointsPerNode) && (_level + 1 <= _maxDepthLevel)) {
		
		//Asignation of values to the childrens
		const CCVector3* point = NULL;
		const ccColor::Rgba* color = NULL;

		std::vector<std::vector <std::vector<glm::vec3> > >pointAuxVector;
		pointAuxVector.resize(2);
		for (int i = 0; i < 2; ++i) {
			pointAuxVector[i].resize(8);
		}

		std::vector<std::vector<std::vector<glm::vec3> > >colorAuxVector;
		colorAuxVector.resize(2);
		for (int i = 0; i < 2; ++i) {
			colorAuxVector[i].resize(8);
		}

		for (int i = 0; i < _numberOfSubclouds; ++i) {
			for (int j = 0; j < _clouds[i]->size(); ++j) {

				point = _clouds[i]->getPoint(j);
				color = &(_clouds[i]->getPointColor(j));

				glm::vec3 coord(point->x, point->y, point->z);
				glm::vec3 colorPoint(color->r, color->g, color->b);

				int index = computeChildrenIndex(coord);

				pointAuxVector[i][index].push_back(coord);
				colorAuxVector[i][index].push_back(colorPoint);
			}
		}

		_clouds[0]->setVisible(false);
		_clouds[1]->setVisible(false);
		
		bool cloudSubdivided = false;
		for (int i = 0; i < 8; ++i) {
			
			unsigned int cloudASize = pointAuxVector[0][i].size();
			unsigned int cloudBSize = pointAuxVector[1][i].size();
			Box corners = computeCorners(i);
			Node* newChild = new Node(this, _cellSize, _numberOfSubclouds, _level + 1, _minimumNumberOfPointsPerNode, _maxDepthLevel, _coordinates, (_squareDim / 2));
			newChild->setCorners(corners.first, corners.second);
			newChild->setAppinterface(_appInterface);
			newChild->addPointsToCloud(pointAuxVector[0][i], colorAuxVector[0][i], 0);
			newChild->addPointsToCloud(pointAuxVector[1][i], colorAuxVector[1][i], 1);
			newChild->computeBoundingBox();
			_childrens.push_back(newChild);

			if ((cloudASize > _minimumNumberOfPointsPerNode) && (cloudBSize > _minimumNumberOfPointsPerNode)) {
				newChild->completeSubdivide();
				cloudSubdivided = true;
			}
			
		}
		if (cloudSubdivided) {
			_clouds[0]->clear();
			_clouds[1]->clear();
		}
	}
}

/**
*	@brief Get all the childrens of a node
*/
std::vector<Node*>* Node::getChildrens()
{
	return &_childrens;
}

/**
*	@brief Get the bounding box of a node
*/
const std::vector<glm::dvec3>& Node::getBoundingBoxVector()
{
	return _boundingBox;
}

/**
*	@brief Set the app interface pointer to a node
*/
void Node::setAppinterface(ccMainAppInterface* appInterface)
{
	_appInterface = appInterface;
}


/**
*	@brief Get the pointer to both clouds in the node
*/
std::vector<ccPointCloud*>* Node::getClouds()
{
	return &_clouds;
}


/**
*	@brief Function to apply the counting box algorithm to a node
*/
void Node::computeCountingBox(Node* node)
{

	node->setComparable(true);
	std::vector<ccPointCloud*>* nodeClouds = node->getClouds();
	ccPointCloud* cloud0 = (*nodeClouds)[0];
	ccPointCloud* cloud1 = (*nodeClouds)[1];
	double squareDim = node->getSquareDim();
	auto bbox = node->getBoundingBoxVector();

	std::vector <double> squareDimVector;

	for (int i = 1; i <= MAX_BOX_COUNTING_RESOLUTION; ++i) {

		double boxDim = squareDim / (NUMBER_OF_SUBDIVISIONS * i);
		squareDimVector.push_back(boxDim);

		for (int j = 0; j < nodeClouds->size(); ++j) {

			std::unordered_set<int> box_hash;	//Create spatial hash for boxes
			box_hash.reserve((*nodeClouds)[j]->size());	//Reserve space for all the points in the cloud

			for (int k = 0; k < (*nodeClouds)[j]->size(); ++k) {	//Iterate through the points


				glm::ivec3 coord = (glm::dvec3((*nodeClouds)[j]->getPoint(k)->x, (*nodeClouds)[j]->getPoint(k)->y, (*nodeClouds)[j]->getPoint(k)->z) - bbox[0]) / boxDim;

				//Generate key
				int key = (coord.x << 20) | (coord.y << 10) | coord.z;

				//Insert key in the spatial hash
				box_hash.insert(key);
			}
			//_boxCounting[j][i - 1] = box_hash.size();	//[subcloud][resolution]
			node->setBoxCountingValue(j, i - 1, box_hash.size());
		}
	}

	for (int i = 0; i < MAX_BOX_COUNTING_RESOLUTION; ++i) {
		double squareDimValue = log((1 / squareDimVector[i]));
		if (squareDimValue == INFINITY) {
			squareDimValue = 0;
		}

		//_squareDimVectorInverseLogarithm[i] = squareDimValue;
		node->setSquareDimVectorInverseLogarithmValue(i, squareDimValue);
	}


}

void Node::assignCountingBox(Node* node)
{


	std::vector<ccPointCloud*>* nodeClouds = node->getClouds();
	ccPointCloud* cloud0 = (*nodeClouds)[0];
	ccPointCloud* cloud1 = (*nodeClouds)[1];

	if ((cloud0->isVisible()) && (cloud1->isVisible()) && (cloud0->size() > 0) && (cloud1->size() > 0)) {
		computeCountingBox(node);
	}
	else {
		std::vector<Node*>* nodeChildrens = node->getChildrens();
		for (int i = 0; i < nodeChildrens->size(); ++i) {
			assignCountingBox((*nodeChildrens)[i]);
		}

		for (int i = 0; i < node->getNumberOfSubclouds(); ++i) {
			unsigned int nonEmptyChildrens = 0;
			for (int j = 0; j < nodeChildrens->size(); ++j) {
				if (getBoxCountingValue((*nodeChildrens)[j], i, 0) != 0) {
					++nonEmptyChildrens;
				}
			}
			node->setBoxCountingValue(i, 0, nonEmptyChildrens);
		}
		node->setSquareDimVectorInverseLogarithmValue(0, node->getSquareDim());


		for (int i = 0; i < node->getNumberOfSubclouds(); ++i) {
			unsigned int childrenBoxes = 0;
			double squareDimValue = 0;
			for (int j = 1; j < MAX_BOX_COUNTING_RESOLUTION; ++j) {
				for (int k = 0; k < nodeChildrens->size(); ++k) {
					childrenBoxes += getBoxCountingValue((*nodeChildrens)[k], i, j - 1);
				}
				node->setBoxCountingValue(i, j, childrenBoxes);
				double previousValue = node->getSquareDimVectorInverseLogarithmValue(node, j - 1);
				node->setSquareDimVectorInverseLogarithmValue(j, previousValue/2);
			}
		}
	}

	if ((cloud0->size() > 0) && (cloud1->size() > 0)){
		std::vector<std::vector<double> > boxCountingLogarithm;
		boxCountingLogarithm.resize(node->getNumberOfSubclouds());
		for (int i = 0; i < node->getNumberOfSubclouds(); ++i) {
			boxCountingLogarithm[i].resize(MAX_BOX_COUNTING_RESOLUTION);
		}

		for (int i = 0; i < node->getNumberOfSubclouds(); ++i) {
			auto BCD = node->getBoxCounting();
			for (int j = 0; j < MAX_BOX_COUNTING_RESOLUTION; ++j) {
				boxCountingLogarithm[i][j] = (double)log(BCD[i][j]);
			}
		}

		const std::vector<double>& squareInverseLogarithm = node->getSquareDimVectorInverseLogarithm();
		double slope0 = getSlope(squareInverseLogarithm, computeLinearRegression(squareInverseLogarithm, boxCountingLogarithm[0]));
		double slope1 = getSlope(squareInverseLogarithm, computeLinearRegression(squareInverseLogarithm, boxCountingLogarithm[1]));
		node->setFD0(slope0);
		node->setFD1(slope1);
		node->setDiff(abs(slope0 - slope1));

	}


	

	//Creation of the scalar field with the fractal dimension differences
	ccScalarField* sf = new ccScalarField("Fractal Dimension Differences");



	int sfIdx0 = cloud0->addScalarField(sf);
	cloud0->setCurrentScalarField(sfIdx0);
	cloud0->setCurrentDisplayedScalarField(sfIdx0);
	cloud0->enableScalarField();

	for (int i = 0; i < cloud0->size(); ++i) {
		cloud0->setPointScalarValue(i, node->getDifferenceBetweenClouds());
	}
	sf->computeMinAndMax();
	cloud0->showSF(true);

	int sfIdx1 = cloud1->addScalarField(sf);
	cloud1->enableScalarField();
	cloud1->setCurrentScalarField(sfIdx1);
	cloud1->setCurrentDisplayedScalarField(sfIdx0);

	for (int i = 0; i < cloud1->size(); ++i) {
		cloud1->setPointScalarValue(i, node->getDifferenceBetweenClouds());
	}
	sf->computeMinAndMax();
	cloud1->showSF(true);

	
}


/**
*	@brief Function to compute a linear regression with the data obtained from the counting box algorithm
*	in order to approximate the result of the fractal dimension
*/
const std::vector<double>& Node::computeLinearRegression(const std::vector<double>& x, const std::vector<double>& y)
{
	double sumX = 0.0, sumX2 = 0.0, sumY = 0.0, sumXY = 0.0, a = 0.0, b = 0.0;
	for (int i = 0; i < MAX_BOX_COUNTING_RESOLUTION; ++i)
	{
		sumX = sumX + x[i];
		sumX2 = sumX2 + x[i] * x[i];
		sumY = sumY + y[i];
		sumXY = sumXY + x[i] * y[i];
	}
	/* Calculating a and b */
	b = (MAX_BOX_COUNTING_RESOLUTION * sumXY - sumX * sumY) / (MAX_BOX_COUNTING_RESOLUTION * sumX2 - sumX * sumX);
	a = (sumY - b * sumX) / MAX_BOX_COUNTING_RESOLUTION;
	
	std::vector<double> yValues;
	for (int i = 0; i < x.size(); ++i) {
		yValues.push_back(a + (b * x[i]));
	}
	return yValues;

}

/**
*	@brief Function to get the slope, which is the fractal dimension, of the function that represents the linear regression from
*	the fractal dimension
*/
double Node::getSlope(const std::vector<double>& x, const std::vector<double>& y)
{
	const auto n = x.size();
	const auto s_x = std::accumulate(x.begin(), x.end(), 0.0);
	const auto s_y = std::accumulate(y.begin(), y.end(), 0.0);
	const auto s_xx = std::inner_product(x.begin(), x.end(), x.begin(), 0.0);
	const auto s_xy = std::inner_product(x.begin(), x.end(), y.begin(), 0.0);
	const auto a = (n * s_xy - s_x * s_y) / (n * s_xx - s_x * s_x);
	return a;
}

/**
*	@brief Get the difference between the clouds from the node
*/
double Node::getDifferenceBetweenClouds()
{
	return _differenceBetweenClouds;
}

/**
*	@brief Function to get if both clouds from the node are comparable
*/
bool Node::areCloudsComparable()
{
	return _areComparable;
}

/**
*	@brief Function to get the depth level of the node
*/
int Node::getLevel()
{
	return _level;
}

unsigned int Node::getNumberOfSubclouds()
{
	return _numberOfSubclouds;
}

const std::vector<std::vector<unsigned int>>& Node::getBoxCounting()
{
	return _boxCounting;
}

const std::vector<double>& Node::getSquareDimVectorInverseLogarithm()
{
	return _squareDimVectorInverseLogarithm;
}

const double& Node::getSquareDim()
{
	return _squareDim;
}

void Node::setFD0(double FD)
{
	_fractalDim0 = FD;
}

void Node::setFD1(double FD)
{
	_fractalDim1 = FD;
}

void Node::setDiff(double diff)
{
	_differenceBetweenClouds = diff;
}

void Node::setComparable(bool comnparable)
{
	_areComparable = true;
}

void Node::setBoxCountingValue(int cloud, int position, unsigned int value)
{
	_boxCounting[cloud][position] = value;
}

void Node::setSquareDimVectorInverseLogarithmValue(int position, double value)
{
	_squareDimVectorInverseLogarithm[position] = value;
}

const unsigned int& Node::getBoxCountingValue(Node* node, int cloud, int position)
{
	const std::vector<std::vector<unsigned int> >& BCDvector = node->getBoxCounting();
	unsigned int value = BCDvector[cloud][position];
	return value;
}

const double& Node::getSquareDimVectorInverseLogarithmValue(Node* node, int position)
{
	const std::vector<double> squareDimVectorInverseLogarithm = node->getSquareDimVectorInverseLogarithm();
	return squareDimVectorInverseLogarithm[position];
}
