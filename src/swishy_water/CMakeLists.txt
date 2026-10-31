cmake_minimum_required(VERSION 3.8)
project(swishy_water)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(gz_sim_vendor REQUIRED)
find_package(gz-sim8 REQUIRED)
find_package(gz_rendering_vendor REQUIRED)
find_package(gz-rendering8 REQUIRED COMPONENTS ogre2)
find_package(gz_plugin_vendor REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
# Ogre-Next headers, needed to reach the shader pass behind Ogre2Material
find_package(gz_ogre_next_vendor REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(OGRE_NEXT REQUIRED IMPORTED_TARGET OGRE-Next OGRE-Next-Hlms)

add_library(swishy-water-shader-system SHARED src/WaterShaderParam.cc)
target_link_libraries(swishy-water-shader-system
  gz-sim8::gz-sim8
  gz-rendering8::gz-rendering8
  gz-rendering8::ogre2
  gz-plugin2::register
  PkgConfig::OGRE_NEXT
)

install(TARGETS swishy-water-shader-system DESTINATION lib)

# put lib/ on GZ_SIM_SYSTEM_PLUGIN_PATH when the workspace is sourced
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/swishy_water.dsv.in")

ament_package()
