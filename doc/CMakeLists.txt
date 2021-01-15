# From https://github.com/TartanLlama/cpp-documentation-example/blob/master/docs/CMakeLists.txt
find_package(Doxygen REQUIRED)
find_package(Sphinx REQUIRED)

set(_pcp_public_header_dir ${PROJECT_SOURCE_DIR}/include/pcp)
file(GLOB_RECURSE _pcp_public_header_files_h 
	${_pcp_public_header_dir}/*.h)
file(GLOB_RECURSE _pcp_public_header_files_hpp 
	${_pcp_public_header_dir}/*.hpp)
set(_pcp_public_header_files)
list(APPEND _pcp_public_header_files 
    ${_pcp_public_header_files_h} 
	${_pcp_public_header_files_hpp})

set(DOXYGEN_INPUT_DIR ${PROJECT_SOURCE_DIR}/include/pcp)
set(DOXYGEN_OUTPUT_DIR ${CMAKE_CURRENT_BINARY_DIR}/doxygen)
set(DOXYGEN_INDEX_FILE ${DOXYGEN_OUTPUT_DIR}/xml/index.xml)
set(DOXYFILE_IN ${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in)
set(DOXYFILE_OUT ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)

# Set the Doxygen input and output directories in the Doxyfile
configure_file(${DOXYFILE_IN} ${DOXYFILE_OUT} @ONLY)

# Doxygen won't create this for us
file(MAKE_DIRECTORY ${DOXYGEN_OUTPUT_DIR}) 

# Only regenerate Doxygen when the Doxyfile or public headers change
add_custom_command(
	OUTPUT ${DOXYGEN_INDEX_FILE}
	DEPENDS ${_pcp_public_header_files}
	COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYFILE_OUT}
	MAIN_DEPENDENCY ${DOXYFILE_OUT} ${DOXYFILE_IN}
	COMMENT "Generating docs"
	VERBATIM)

# Nice named target so we can run the job easily
add_custom_target(pcp-doxygen ALL DEPENDS ${DOXYGEN_INDEX_FILE})

set(SPHINX_SOURCE ${CMAKE_CURRENT_SOURCE_DIR})
set(SPHINX_BUILD ${CMAKE_CURRENT_BINARY_DIR}/sphinx)
set(SPHINX_INDEX_FILE ${SPHINX_BUILD}/index.html)

file(GLOB_RECURSE _pcp_doc_files_rst 
	${CMAKE_CURRENT_SOURCE_DIR}/*.rst)

# Only regenerate Sphinx when:
#  - Doxygen has rerun 
#  - Our doc files have been updated
#  - The Sphinx config has been updated
add_custom_command(
	OUTPUT ${SPHINX_INDEX_FILE}
	COMMAND 
		${SPHINX_EXECUTABLE} -b html
		# Tell Breathe where to find the Doxygen output
		-Dbreathe_projects.pcp=${DOXYGEN_OUTPUT_DIR}/xml
		${SPHINX_SOURCE} ${SPHINX_BUILD}
	WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
	DEPENDS 
		# Other docs files you want to track should go here (or in some variable)
		${_pcp_doc_files_rst}
		${DOXYGEN_INDEX_FILE}
	MAIN_DEPENDENCY ${SPHINX_SOURCE}/conf.py
	COMMENT "Generating documentation with Sphinx")

# Nice named target so we can run the job easily
add_custom_target(pcp-sphinx ALL DEPENDS ${SPHINX_INDEX_FILE})

include(GNUInstallDirs)
install(
	DIRECTORY ${SPHINX_BUILD}
	DESTINATION ${CMAKE_INSTALL_DOCDIR})