cmake_minimum_required(VERSION 2.8.3)
project(data_exportor)

find_package(catkin REQUIRED COMPONENTS
  rosbag
)

catkin_package(
  # INCLUDE_DIRS include
  CATKIN_DEPENDS
)

###########
## Build ##
###########

include_directories(
  # include
  # ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

# Python scripts
# set(PYTHON_SCRIPTS
# )

# foreach(scripts ${PYTHON_SCRIPTS})
#   catkin_install_python(PROGRAMS ${scripts}
#     DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
# endforeach()

#############
## Testing ##
#############

if (CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)

  set(DIRECTORIES
    ./launch/
    )

  foreach(dirs ${DIRECTORIES})
    roslaunch_add_file_check(${dirs})
  endforeach()
endif()