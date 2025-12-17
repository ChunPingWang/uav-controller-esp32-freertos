# Fixed-Wing Aircraft Controller
# ESP-IDF Project Root CMakeLists.txt

cmake_minimum_required(VERSION 3.16)

# Include component directories
set(EXTRA_COMPONENT_DIRS
    "${CMAKE_CURRENT_SOURCE_DIR}/components"
)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

project(fixed_wing_controller)
