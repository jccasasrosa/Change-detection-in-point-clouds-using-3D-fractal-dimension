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
Node::Node(Node* parent, float cellSize, const unsigned int& numberOfSubclouds, int level)
{
	_parent = parent;
	_cellSize = cellSize;
	_numberOfSubclouds = numberOfSubclouds;
	_level = level;

	for (int i = 0; i < numberOfSubclouds; ++i) {
		_clouds.push_back(new ccPointCloud());
	}

	_numberOfPoints = 0;
	_appInterface = 0;

	_boxCounting.resize(numberOfSubclouds);
	for (int i = 0; i < numberOfSubclouds; ++i) {
		_boxCounting[i].resize(MAX_BOX_COUNTING_RESOLUTION);
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
	_boundingBox.push_back(glm::vec3(FLT_MAX, FLT_MAX, FLT_MAX));	//Min values
	_boundingBox.push_back(glm::vec3(FLT_MIN, FLT_MIN, FLT_MIN));	//Max values
	const CCVector3* point = NULL;

	long pointCount = 0;
	
	for (int i = 0; i < _clouds.size(); ++i) {
		for (int j = 0; j < _clouds[i]->size(); ++j) {

			point = _clouds[i]->getPoint(j);

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
		pointCount += _clouds[i]->size();
	}
	_numberOfPoints = pointCount;
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
	glm::vec3 center = glm::vec3((_boundingBox[0].x + _boundingBox[1].x) / 2, (_boundingBox[0].y + _boundingBox[1].y) / 2, (_boundingBox[0].z + _boundingBox[1].z) / 2);


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

	return index;

}

/**
*	@brief Subdivide once both clouds are loaded
*/
void Node::completeSubdivide()
{

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

		
	
	for (int i = 0; i < 8; ++i) {
		unsigned int cloudASize = pointAuxVector[0][i].size();
		unsigned int cloudBSize = pointAuxVector[1][i].size();
		if ((cloudASize > MIN_NUMBER_OF_POINTS_PER_NODE) && (cloudBSize > MIN_NUMBER_OF_POINTS_PER_NODE)) {
			Node* newChild = new Node(this, _cellSize, _numberOfSubclouds, _level + 1);
			newChild->setAppinterface(_appInterface);
			newChild->addPointsToCloud(pointAuxVector[0][i], colorAuxVector[0][i], 0);
			newChild->addPointsToCloud(pointAuxVector[1][i], colorAuxVector[1][i], 1);
			newChild->computeBoundingBox();
			newChild->completeSubdivide();
			_childrens.push_back(newChild);
		}
	}

	if (_childrens.size() == 8) {
		_clouds[0]->setVisible(false);
		_clouds[1]->setVisible(false);
	}
	
}

/**
*	@brief Get all the childrens of a node
*/
const std::vector<Node*>& Node::getChildrens()
{
	return _childrens;
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
void Node::computeCountingBox()
{
	if ((_clouds[0]->size() > 0) && (_clouds[1]->size() > 0)) {

		_areComparable = true;	//In this case, both clouds are comparable


		double originalBoxDimX = abs(_boundingBox[1].x - _boundingBox[0].x);
		double originalBoxDimY = abs(_boundingBox[1].y - _boundingBox[0].y);
		double originalBoxDimZ = abs(_boundingBox[1].z - _boundingBox[0].z);

		double squareDim = originalBoxDimX > originalBoxDimY ? originalBoxDimX : originalBoxDimY;
		squareDim = squareDim > originalBoxDimZ ? squareDim : originalBoxDimZ;

		std::vector <double> squareDimVector;

		for (int i = 1; i <= MAX_BOX_COUNTING_RESOLUTION; ++i) {

			double boxDim = squareDim / (NUMBER_OF_SUBDIVISIONS * i);
			squareDimVector.push_back(boxDim);

			for (int j = 0; j < _clouds.size(); ++j) {

				std::unordered_set<int> box_hash;	//Create spatial hash for boxes
				box_hash.reserve(_clouds[j]->size());	//Reserve space for all the points in the cloud

				for (int k = 0; k < _clouds[j]->size(); ++k) {	//Iterate through the points

					double coordX = _clouds[j]->getPoint(k)->x - _boundingBox[0].x;
					double coordY = _clouds[j]->getPoint(k)->y - _boundingBox[0].y;
					double coordZ = _clouds[j]->getPoint(k)->z - _boundingBox[0].z;


					//Generate key
					int key = ((int)(coordX / boxDim) << 20) |
						((int)(coordY / boxDim) << 10) |
						(int)(coordZ / boxDim);

					//Insert key in the spatial hash
					box_hash.insert(key);
				}
				_boxCounting[j][i - 1] = box_hash.size();	//[subcloud][resolution]
			}
		}

		//Creation and computation of the logarithm of each element

		std::vector<std::vector<double> > boxCountingLogarithm;
		boxCountingLogarithm.resize(_numberOfSubclouds);
		for (int i = 0; i < _numberOfSubclouds; ++i) {
			boxCountingLogarithm[i].resize(MAX_BOX_COUNTING_RESOLUTION);
		}

		std::vector<double> squareDimVectorInverseLogarithm;
		squareDimVectorInverseLogarithm.resize(MAX_BOX_COUNTING_RESOLUTION);
		for (int i = 0; i < MAX_BOX_COUNTING_RESOLUTION; ++i) {
			double squareDimValue = 1 / log(squareDimVector[i]);
			if (squareDimValue == INFINITY) {
				squareDimValue = 0;
			}
			squareDimVectorInverseLogarithm[i] = squareDimValue;
		}

		for (int i = 0; i < _numberOfSubclouds; ++i) {
			for (int j = 0; j < MAX_BOX_COUNTING_RESOLUTION; ++j) {
				boxCountingLogarithm[i][j] = (double)log(_boxCounting[i][j]);
			}
		}

		double slope0 = getSlope(squareDimVectorInverseLogarithm, computeLinearRegression(squareDimVectorInverseLogarithm, boxCountingLogarithm[0]));
		double slope1 = getSlope(squareDimVectorInverseLogarithm, computeLinearRegression(squareDimVectorInverseLogarithm, boxCountingLogarithm[1]));

		_fractalDim0 = slope0;
		_fractalDim1 = slope1;
		_differenceBetweenClouds = abs(slope0 - slope1);

	}

	//Creation of the scalar field with the fractal dimension differences
	ccScalarField* sf = new ccScalarField("Fractal Dimension Differences");

	int sfIdx0 = _clouds[0]->addScalarField(sf);
	_clouds[0]->setCurrentScalarField(sfIdx0);
	_clouds[0]->setCurrentDisplayedScalarField(sfIdx0);
	_clouds[0]->enableScalarField();

	for (int i = 0; i < _clouds[0]->size(); ++i) {
		_clouds[0]->setPointScalarValue(i, _differenceBetweenClouds);
	}
	//_clouds[0]->showSF(true);

	int sfIdx1 = _clouds[1]->addScalarField(sf);
	_clouds[1]->enableScalarField();
	_clouds[1]->setCurrentScalarField(sfIdx1);
	_clouds[1]->setCurrentDisplayedScalarField(sfIdx0);

	for (int i = 0; i < _clouds[1]->size(); ++i) {
		_clouds[1]->setPointScalarValue(i, _differenceBetweenClouds);
	}

	for (auto* child : _childrens) {
		child->computeCountingBox();
	}
}

/**
*	@brief Function to apply the couning box algorithm to the children and recursively (preorder)
*/
void Node::computeCountingBoxChildren()
{
	for (auto* child : _childrens) {
		child->computeCountingBox();
		child->computeCountingBoxChildren();
	}
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
	for (double x0 : x) {
		yValues.push_back(a + (b * x0));
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

/**
*	@brief Store the poins from both clouds from the node in the file system
*/
void Node::storePoints(int index)
{
	if ((_clouds[0]->size() > 0) && (_clouds[1]->size() > 0)) 
	{
		LASheader header;
		header.x_scale_factor = 0.1;
		header.y_scale_factor = 0.01;
		header.z_scale_factor = 0.001;
		header.x_offset = 1000.0;
		header.y_offset = 1000.0;
		header.z_offset = 1000.0;
		header.point_data_format = 3;
		header.point_data_record_length = 36;

		LASwriteOpener laswriteopener;
		LASwriter* laswriter;


		LASpoint point;
		point.init(&header, header.point_data_format, header.point_data_record_length, 0);

		for (int i = 0; i < _numberOfSubclouds; ++i) {
			std::string route = "C:\\Users\\UJA\\Desktop\\Octrees_3\\" + std::to_string(index);
			fs::create_directory(route);
			std::string filename = route + "\\Cloud_" + std::to_string(i) + ".laz";
			laswriteopener.set_file_name(filename.c_str());
			laswriter = laswriteopener.open(&header);

			for (int j = 0; j < _clouds[i]->size(); ++j) {

				point.set_x(_clouds[i]->getPoint(j)->x);
				point.set_y(_clouds[i]->getPoint(j)->y);
				point.set_z(_clouds[i]->getPoint(j)->z);
				U16 rgbcolor[3] = { _clouds[i]->getPointColor(j).r * 256, _clouds[i]->getPointColor(j).g * 256, _clouds[i]->getPointColor(j).b * 256 };

				point.set_RGB(rgbcolor);

				laswriter->write_point(&point);
				laswriter->update_inventory(&point);
			}

			laswriter->close();
			delete laswriter;

		}
	}
}
