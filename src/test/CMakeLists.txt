include_directories(
        ${CMAKE_SOURCE_DIR}/src
        ${CMAKE_BINARY_DIR}/src
)

file(GLOB header
        "*.h"
        )
file(GLOB src
        "*.cpp"
        )
configure_file (
        "Config.h.in"
        "Config.h"
)

add_executable(${PROJECT_NAME}_Test ${header} ${src})
set_property(TARGET ${PROJECT_NAME}_Test PROPERTY CXX_STANDARD 14)
target_link_libraries(${PROJECT_NAME}_Test ${Boost_LIBRARIES})
#add_dependencies(${PROJECT_NAME}_Test ${PROJECT_NAME})
#target_link_libraries(${PROJECT_NAME}_Test ${PROJECT_NAME} ${Boost_LIBRARIES})
