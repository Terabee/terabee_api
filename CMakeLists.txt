cmake_minimum_required(VERSION 3.0.2)
project(TerabeeApi CXX)

set(CMAKE_CXX_STANDARD 14)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Werror -O -g -pthread -fPIC")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/build/bin)

set(LIB_NAME terabeeapi)

option(BUILD_TESTS "Builds tests" ON)
option(BUILD_EXAMPLES "Build short example binaries" ON)
option(BUILD_SHARED_LIBS
  "Will build shared libraries if ON; recommended to switch it ON when working on Linux"
  ON
)

find_package(spdlog 1.4.2 QUIET)

if (NOT spdlog_FOUND)
  message(STATUS "Need to build spdlog")
  add_subdirectory(3rdParty/spdlog-1.4.2)
endif()

add_library(${LIB_NAME}
  src/factories/FactoryGetters.cpp
  src/factories/TerarangerFactory.cpp
  src/logger/Logger.cpp
  src/ring_buffer/RingBuffer.cpp
  src/serial_communication/Serial.cpp
  src/teraranger/TerarangerBasicCommon.cpp
  src/teraranger/TerarangerEvo3m.cpp
  src/teraranger/TerarangerEvo600Hz.cpp
  src/teraranger/TerarangerEvo60m.cpp
  src/teraranger/TerarangerEvo64px.cpp
  src/teraranger/TerarangerEvoMini.cpp
  src/teraranger/TerarangerEvoThermal33.cpp
  src/teraranger/TerarangerEvoThermal90.cpp
  src/teraranger/TerarangerMultiflex.cpp
  src/teraranger/TerarangerThermalCommon.cpp
  src/teraranger/TerarangerTowerEvo.cpp
)
target_include_directories(${LIB_NAME} PUBLIC include)
target_link_libraries(${LIB_NAME} spdlog::spdlog)

if (BUILD_EXAMPLES)
  add_subdirectory(examples)
endif()

if(BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()

# sudo make install
# then find_package(TerabeeApi REQUIRED)
# and target_link_libraries(myBin ${TerabeeApi_LIBRARIES})
set(INSTALL_LIB_DIR lib CACHE PATH "Installation directory for libraries")
set(INSTALL_BIN_DIR bin CACHE PATH "Installation directory for executables")
set(INSTALL_INCLUDE_DIR include CACHE PATH "Installation directory for header files")
set(INSTALL_CMAKE_DIR lib/cmake/TerabeeApi CACHE PATH "Installation directory for CMake files")

install(DIRECTORY include/terabee DESTINATION ${INSTALL_INCLUDE_DIR}
    PATTERN "include/terabee/internal" EXCLUDE)
install(TARGETS ${LIB_NAME}
  LIBRARY DESTINATION ${INSTALL_LIB_DIR}
  ARCHIVE DESTINATION ${INSTALL_LIB_DIR}
  RUNTIME DESTINATION ${INSTALL_BIN_DIR})

if (BUILD_SHARED_LIBS)
  set(NATIVE_LIB_NAME ${CMAKE_SHARED_LIBRARY_PREFIX}${LIB_NAME}${CMAKE_SHARED_LIBRARY_SUFFIX})
else(BUILD_SHARED_LIBS)
  set(NATIVE_LIB_NAME ${CMAKE_STATIC_LIBRARY_PREFIX}${LIB_NAME}${CMAKE_STATIC_LIBRARY_SUFFIX})
endif()
set(TerabeeApi_LIB "${CMAKE_INSTALL_PREFIX}/${INSTALL_LIB_DIR}/${NATIVE_LIB_NAME}")

configure_file(cmake/TerabeeApiConfig.cmake.in
  "${PROJECT_BINARY_DIR}/TerabeeApiConfig.cmake" @ONLY)

install(FILES "${PROJECT_BINARY_DIR}/TerabeeApiConfig.cmake"
  DESTINATION ${INSTALL_CMAKE_DIR})
# compatibility with catkin
install (FILES "${PROJECT_BINARY_DIR}/TerabeeApiConfig.cmake"
  DESTINATION "share/${PROJECT_NAME}/cmake/"
)
