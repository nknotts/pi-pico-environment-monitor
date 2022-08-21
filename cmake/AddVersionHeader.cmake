
find_package(Git REQUIRED)

set( _ADD_VERSION_HEADER_MODULE_DIR ${CMAKE_CURRENT_LIST_DIR})


function(AddVersionHeader targetname libname)

	add_library(${targetname} INTERFACE)

	add_custom_target(${targetname}_version_header ALL)
	add_custom_command(TARGET ${targetname}_version_header
		COMMAND ${CMAKE_COMMAND}
		-Dlibname=${libname}
		-DTEMPLATE_FILE=${_ADD_VERSION_HEADER_MODULE_DIR}/version.h.in
		-DGIT_WORKING_DIR=${CMAKE_CURRENT_SOURCE_DIR}
		-DGIT_EXECUTABLE=${GIT_EXECUTABLE}
		-DBUILD_HOST=${BUILD_HOST}
		-DOUTPUT_FILE=${CMAKE_CURRENT_BINARY_DIR}/${targetname}/lib${libname}_version.h
		-P ${_ADD_VERSION_HEADER_MODULE_DIR}/MakeVersionHeader.cmake)

	add_dependencies(${targetname} ${targetname}_version_header)

	target_include_directories(${targetname} INTERFACE
		$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/${targetname}>)

endfunction()
