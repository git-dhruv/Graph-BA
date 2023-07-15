cmake_minimum_required(VERSION 2.8)

project(bundle_adjustment)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-O3 -std=c++17")

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

# Find_Package(G2O REQUIRED)
Find_Package(Eigen3 REQUIRED)
Find_Package(Ceres REQUIRED)
Find_Package(Sophus REQUIRED)
find_package( OpenCV REQUIRED )

# Find_Package(CSparse REQUIRED)

# SET(G2O_LIBS g2o_csparse_extension g2o_stuff g2o_core cxsparse)

include_directories(${PROJECT_SOURCE_DIR} ${EIGEN3_INCLUDE_DIR} ${CSPARSE_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS})

# add_library(bal_common common.cpp)
# add_executable(bundle_adjustment_g2o bundle_adjustment_g2o.cpp)
add_executable(main main.cpp)
target_link_libraries(main ${OpenCV_LIBS} ${PROJECT_SOURCE_DIR} ${EIGEN3_INCLUDE_DIR} ${CERES_LIBRARIES})
# target_link_libraries(bundle_adjustment_ceres ${CERES_LIBRARIES} bal_common)
# target_link_libraries(bundle_adjustment_g2o ${G2O_LIBS} bal_common)

# add_library( # Sets the name of the library.
#              ceres-lib )
# target_compile_definitions(ceres-lib PUBLIC CERES_USE_CXX11_THREADS=1)