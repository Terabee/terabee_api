add_executable(ExampleAsyncAdvanced ExampleAsyncAdvanced.cpp)
target_link_libraries(ExampleAsyncAdvanced ${LIB_NAME})

add_executable(ExampleAsyncSimple ExampleAsyncSimple.cpp)
target_link_libraries(ExampleAsyncSimple ${LIB_NAME})

add_executable(ExampleReadEvo3m ExampleReadEvo3m.cpp)
target_link_libraries(ExampleReadEvo3m ${LIB_NAME})

add_executable(ExampleReadEvo600Hz ExampleReadEvo600Hz.cpp)
target_link_libraries(ExampleReadEvo600Hz ${LIB_NAME})

add_executable(ExampleReadEvo15m ExampleReadEvo15m.cpp)
target_link_libraries(ExampleReadEvo15m ${LIB_NAME})

add_executable(ExampleReadEvo40m ExampleReadEvo40m.cpp)
target_link_libraries(ExampleReadEvo40m ${LIB_NAME})

add_executable(ExampleReadEvo60m ExampleReadEvo60m.cpp)
target_link_libraries(ExampleReadEvo60m ${LIB_NAME})

add_executable(ExampleReadEvo64px ExampleReadEvo64px.cpp)
target_link_libraries(ExampleReadEvo64px ${LIB_NAME})

add_executable(ExampleReadEvoMini ExampleReadEvoMini.cpp)
target_link_libraries(ExampleReadEvoMini ${LIB_NAME})

find_package(OpenCV 3 QUIET)
if (OpenCV_FOUND)
  add_executable(ExampleReadEvoThermal ExampleReadEvoThermal.cpp)
  target_link_libraries(ExampleReadEvoThermal ${LIB_NAME} ${OpenCV_LIBRARIES})
endif()

add_executable(ExampleReadMultiflex ExampleReadMultiflex.cpp)
target_link_libraries(ExampleReadMultiflex ${LIB_NAME})

add_executable(ExampleReadTowerEvo ExampleReadTowerEvo.cpp)
target_link_libraries(ExampleReadTowerEvo ${LIB_NAME})
