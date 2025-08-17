cmake_minimum_required(VERSION 3.8)
project(lightrover_description)

find_package(ament_cmake REQUIRED)

# === share へリソースをインストール（存在するものだけ） ===
foreach(dir urdf launch rviz worlds config meshes)
  if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${dir})
    install(DIRECTORY ${dir}/
      DESTINATION share/${PROJECT_NAME}/${dir})
  endif()
endforeach()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
