cmake_minimum_required(VERSION 2.8.3)
project(imu_sensor)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  rospy
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES imu_sensor
#  CATKIN_DEPENDS rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  )

  # Python scripts
  set(PYTHON_SCRIPTS
    scripts/mpu6050.py
  )
  
  foreach(scripts ${PYTHON_SCRIPTS})
    catkin_install_python(PROGRAMS ${scripts}
      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
  endforeach()

set(DIRECTORIES
  ./launch/
  )

# other files
foreach(dirs ${DIRECTORIES})
   install(DIRECTORY ${dirs}
      DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dirs})
endforeach()

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