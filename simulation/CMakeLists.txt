include_directories(include)

add_executable(points_generation src/points_generation.cpp)
target_link_libraries(points_generation ${yaml-cpp_LIBRARIES} yaml-cpp ${gflags_LIBRARIES})

add_executable(image_generation src/image_generation.cpp)
target_link_libraries(image_generation ${yaml-cpp_LIBRARIES} yaml-cpp ${gflags_LIBRARIES} ${OpenCV_LIBS})