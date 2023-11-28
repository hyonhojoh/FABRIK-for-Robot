cmake_minimum_required(VERSION 2.8.3)

project(a_fabrik)

find_package(catkin REQUIRED COMPONENTS roscpp)
find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)

catkin_package()

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${Eigen_INCLUDE_DIRS}
)

add_executable(main src/main.cpp)
target_link_libraries(main ${catkin_LIBRARIES} Eigen3::Eigen pinocchio::pinocchio)

add_executable(RMSE src/RMSE.cpp)
target_link_libraries(RMSE ${catkin_LIBRARIES} Eigen3::Eigen)

