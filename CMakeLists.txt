cmake_minimum_required(VERSION 3.0.2)
project(compression)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -g -Wextra")
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge           # To convert between pixel formats
  image_transport     # To publish and subscribe images
  message_generation  # To build ImagePacket msg
  roscpp              # Standard
  sensor_msgs         # Image msg
  std_msgs            # Header msg
)

set(cv_bridge_DIR /opt/ros/noetic/share/cv_bridge/cmake)

find_package(FFMPEG REQUIRED)
find_package(OpenCV REQUIRED)

# Tell CMake to include the custom message (msg/ImagePacket.msg)
# and generate the source file (ImagePacket.h)
add_message_files(DIRECTORY msg FILES ImagePacket.msg)
generate_messages(DEPENDENCIES std_msgs compression)

include_directories(
  include
  ${FFMPEG_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# Specify that this project is a package and include it's dependencies
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS message_runtime std_msgs roscpp cv_bridge image_transport sensor_msgs
  DEPENDS FFMPEG
)

# Create the executables and add their sources
add_executable(h265_compressor_node_executable src/h265_compressor_node.cpp)
add_executable(h265_decoder_node_executable src/h265_decoder_node.cpp)

# Link the executables and libraries
target_link_libraries(h265_compressor_node_executable 
  ${catkin_LIBRARIES}
  ${FFMPEG_LIBRARIES}
)

# Link the executables and libraries
target_link_libraries(h265_decoder_node_executable
  ${catkin_LIBRARIES}
  ${FFMPEG_LIBRARIES}
)