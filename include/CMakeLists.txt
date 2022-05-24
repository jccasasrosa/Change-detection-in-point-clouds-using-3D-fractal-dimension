
target_sources( ${PROJECT_NAME}
	PRIVATE
		${CMAKE_CURRENT_LIST_DIR}/ActionA.h
		${CMAKE_CURRENT_LIST_DIR}/Node.h
		${CMAKE_CURRENT_LIST_DIR}/Octree.h
		${CMAKE_CURRENT_LIST_DIR}/OctreeGrid.h
		${CMAKE_CURRENT_LIST_DIR}/TemporalSeriesPlugin.h
		${CMAKE_CURRENT_LIST_DIR}/TemporalSeriesPluginInterface.h
)

target_include_directories( ${PROJECT_NAME}
	PRIVATE
		${CMAKE_CURRENT_SOURCE_DIR}
)
