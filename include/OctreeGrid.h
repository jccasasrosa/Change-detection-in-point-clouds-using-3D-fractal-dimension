#include "glm.hpp"
#include <unordered_map>
#include "Octree.h"
#include <ccPointCloud.h>
#include <ccPolyline.h>
#include <ccMainAppInterface.h>
#define CELL_SIZE 50


//Hash function of our octree grid
namespace glm {
	template <typename T> struct tvec3Hash {
		std::size_t operator()(const glm::tvec3<T>& key) const {
			return key.x * 18397 + key.y * 20483 + key.z * 29303;
		}
	};
	using ivec3Hash = tvec3Hash<int>;
	using fvec3Hash = tvec3Hash<float>;
	using dvec3Hash = tvec3Hash<double>;
}


class OctreeGrid {
private:
	std::unordered_map<glm::ivec3, Octree*, glm::fvec3Hash> _octrees;	//Sparse grid of octrees
	float _cellSize;	//Size of each cell
	int _minimumNumberOfPointsPerNode;	//Minimum number of points per node
	int _maxDepthLevel;	//Maximum octree depth level
	std::vector<glm::dvec3> _boundingBox;	//Bounding box of the point cloud 
	bool _octreeButtonToggled;
	bool _nodeButtonToggled;
	bool _polilynesCreated;

	std::vector<ccPointCloud*> _octreesAuxCloud;
	std::vector<ccPolyline*> _octreesPolilynes;

	std::vector<ccPointCloud*> _nodesAuxCloud;
	std::vector<ccPolyline*> _nodesPolilynes;

	ccMainAppInterface* _appInterface;

	std::unordered_map<glm::ivec3, std::vector<unsigned int> , glm::ivec3Hash> _indexStructure;
	std::vector<Octree*> _octreesForParallelization;

public:
	OctreeGrid();
	OctreeGrid(float cellSize, int minimumNumberOfPointsPerNode, int maxDepthLevel);
	void computeBoundingBox(const std::vector<glm::vec3>& pointsPositions);
	void loadOctreeGrid(const std::vector<glm::vec3>& pointsPositions, const std::vector<glm::vec3>& pointsColors, const unsigned int& numberOfClouds, const unsigned int& subcloud);
	const std::vector<glm::dvec3>& getBoundingBoxVector();
	const std::unordered_map<glm::ivec3, Octree*, glm::fvec3Hash>& getOctreeGrid();
	void completeSubdivide();
	const bool& getOctreeButtonState();
	const bool& getNodeButtonState();

	void changeOctreeState();
	void changeNodeState();

	void setAppinterface(ccMainAppInterface* appInterface);
	
	void auxComputeVisualization(std::vector<ccPointCloud*>* pointCloudVector, std::vector<ccPolyline*>* polilyneVector, const std::vector<glm::dvec3>& boundingBoxVector, const ccColor::Rgba& color);
	void computeOctreeVisualization();
	void computeNodeVisualization(Node* node);
	void computeChildVisualization(Node* node);

	void fillOctreesVector();

	void computeCountingBox();

	void changeOctreeVisibility(const bool& state);
	void changeNodesVisibility(const bool& state);



};