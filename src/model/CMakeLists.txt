cmake_minimum_required(VERSION 3.8)
project(nav_simplified_model)

find_package(ament_cmake REQUIRED)

foreach(dir config launch maps meshes textures urdf)
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${dir}")
    install(
      DIRECTORY ${dir}/
      DESTINATION share/${PROJECT_NAME}/${dir}
    )
  endif()
endforeach()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/scripts/fake_base_odom.py")
  install(
    PROGRAMS
      scripts/fake_base_odom.py
      scripts/twist_to_twist_stamped.py
      scripts/waypoint_loop_runner.py
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

ament_package()
