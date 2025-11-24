cmake_minimum_required(VERSION 3.10)
project(adam_lib)


add_library(adam_lib SHARED
  Core/renderer.cpp
  Core/window.cpp
  Core/AdamLib.cpp
  Core/inputs.cpp

  Nodes/node.cpp
  Nodes/spritenode.cpp
  Nodes/collisionnode.cpp

  Collision/AABB.cpp
  Collision/collisiondetector.cpp
  Collision/collisionshapes.cpp
  Collision/cute_c2_impl.cpp

  Resources/manager.cpp
  Resources/textures.cpp

  Utilities/math.cpp
  Utilities/signal.cpp
        Collision/CollisionDetectorInternal.hpp
)

target_include_directories(adam_lib
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

add_library(adam::lib ALIAS adam_lib)

target_link_libraries(adam_lib
  PUBLIC
    SDL3::SDL3
    $<IF:$<TARGET_EXISTS:SDL3_image::SDL3_image-shared>,
       SDL3_image::SDL3_image-shared, SDL3_image::SDL3_image-static>
    $<IF:$<TARGET_EXISTS:SDL3_ttf::SDL3_ttf-shared>,
       SDL3_ttf::SDL3_ttf-shared, SDL3_ttf::SDL3_ttf-static>
)
