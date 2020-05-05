add_executable(ut-ring_buffer
  RingBufferTests.cpp
)
target_link_libraries(ut-ring_buffer gmock_main ${LIB_NAME})
add_test(RingBufferTests ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/ut-ring_buffer)
