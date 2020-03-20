include_directories(${SCALFU_INCLUDE_DIRS})
link_directories(${SCALFU_LIBRARY_DIRS})
message("${SCALFU_INCLUDE_DIRS}")

add_executable(reconstruction
        reconstruction.cpp
        )

target_link_libraries(reconstruction
        ${SCALFU_LIBRARIES})




