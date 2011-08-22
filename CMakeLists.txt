# This project is designed to be built outside the Insight source tree.
PROJECT(Strategy)
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

# Find ITK
FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

# Setup paths
SET(DATA_PATH ${Strategy_SOURCE_DIR}/Data)
SET(SOURCE_PATH ${Strategy_SOURCE_DIR}/Source)
SET(TESTING_PATH ${Strategy_SOURCE_DIR}/Testing)

# Include source path
INCLUDE_DIRECTORIES(
  BEFORE
  ${SOURCE_PATH}
  ${TESTING_PATH}
  )

# Process sub-directories
ADD_SUBDIRECTORY(Source)
OPTION(BUILD_TESTING "Build the Testing directory." ON)
IF(BUILD_TESTING)
  ENABLE_TESTING()
  SUBDIRS(Testing)
ENDIF(BUILD_TESTING)
