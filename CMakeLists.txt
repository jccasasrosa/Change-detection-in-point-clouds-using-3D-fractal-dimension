# CloudCompare example for standard plugins

# REPLACE ALL 'TemporalSeriesPlugin' OCCURENCES BY YOUR PLUGIN NAME
# AND ADAPT THE CODE BELOW TO YOUR OWN NEEDS!

# Add an option to CMake to control whether we build this plugin or not
option( PLUGIN_TEMPORAL_SERIES "Install Temporal Series plugin" ON )

if ( PLUGIN_TEMPORAL_SERIES )
	project( TemporalSeriesPlugin )
	 
	AddPlugin( NAME ${PROJECT_NAME} )
	set(CMAKE_AUTOUIC ON)
		
	add_subdirectory( include )
	add_subdirectory( src )
	add_subdirectory( ui )
	include_directories( $CloudCompare_SOURCE_DIR} )
	
	# set dependencies to necessary libraries
	# target_link_libraries( ${PROJECT_NAME} LIB1 )
endif()
