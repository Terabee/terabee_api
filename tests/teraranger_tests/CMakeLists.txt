add_executable(ut-evo64px
  TerarangerEvo64pxTests.cpp
)
target_link_libraries(ut-evo64px gmock_main terabee_mocks ${LIB_NAME})
add_test(Evo64pxTests ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/ut-evo64px)

add_executable(ut-evo_basic_common
  TerarangerEvoBasicCommonTests.cpp
)
target_link_libraries(ut-evo_basic_common gmock_main terabee_mocks ${LIB_NAME})
add_test(EvoBasicCommonTests ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/ut-evo_basic_common)

add_executable(ut-evo_mini
  TerarangerEvoMiniTests.cpp
)
target_link_libraries(ut-evo_mini gmock_main terabee_mocks ${LIB_NAME})
add_test(EvoMiniTests ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/ut-evo_mini)

add_executable(ut-evo_thermal_common
  TerarangerEvoThermalCommonTests.cpp
)
target_link_libraries(ut-evo_thermal_common gmock_main terabee_mocks ${LIB_NAME})
add_test(EvoThermalCommonTests ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/ut-evo_thermal_common)

add_executable(ut-multiflex
  TerarangerMultiflexTests.cpp
)
target_link_libraries(ut-multiflex gmock_main terabee_mocks ${LIB_NAME})
add_test(MultiflexTests ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/ut-multiflex)


add_executable(ut-tower_evo
  TerarangerTowerEvoTests.cpp
)
target_link_libraries(ut-tower_evo gmock_main terabee_mocks ${LIB_NAME})
add_test(TowerEvoTests ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/ut-tower_evo)
