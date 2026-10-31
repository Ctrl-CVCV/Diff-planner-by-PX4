cmake_minimum_required(VERSION 3.0.2)
project(px4_servo)

find_package(catkin REQUIRED)

catkin_package()

# servo_map.py 是节点导入的纯函数模块，必须一起装进 devel/lib，否则运行时 ImportError
catkin_install_python(PROGRAMS
  scripts/servo_aux_pwm.py
  scripts/servo_map.py
  scripts/servo_watch.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(tests/test_servo_map.py)
endif()
