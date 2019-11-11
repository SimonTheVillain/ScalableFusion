
include_directories(${SCALFU_INCLUDE_DIRS})
link_directories(${SCALFU_LIBRARY_DIRS})

add_executable(graph_deformation_test
        graph_deformation_test.cpp
        )

target_link_libraries(graph_deformation_test
        ${SCALFU_LIBRARIES})


add_executable(label_cloud_viewer
        label_cloud_viewer.cpp
        )

target_link_libraries(label_cloud_viewer
        ${SCALFU_LIBRARIES})



add_executable(simple_cloud_viewer
        simple_cloud_viewer.cpp
        )

target_link_libraries(simple_cloud_viewer
        ${SCALFU_LIBRARIES})


add_executable(texture_mem_test
        texture_mem_test.cpp
        )

target_link_libraries(texture_mem_test
        ${SCALFU_LIBRARIES})



add_executable(texture_mem_test_threaded
        texture_mem_test_threaded.cpp
        )

target_link_libraries(texture_mem_test_threaded
        ${SCALFU_LIBRARIES})




