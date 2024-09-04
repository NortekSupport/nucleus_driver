cmake_minimum_required(VERSION 3.8)
project(interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ConnectSerial.srv"
  "srv/ConnectTcp.srv"
  "srv/Disconnect.srv"
  "srv/Start.srv"
  "srv/StartFieldCalibration.srv"
  "srv/Stop.srv"
  "srv/Command.srv"
  
  "msg/AHRS.msg"
  "msg/Altimeter.msg"
  "msg/BottomTrack.msg"
  "msg/WaterTrack.msg"
  "msg/CurrentProfile.msg"
  "msg/FieldCalibration.msg"
  "msg/IMU.msg"
  "msg/INS.msg"
  "msg/Magnetometer.msg"
  DEPENDENCIES builtin_interfaces
)

ament_package()
