file(GLOB header "*.h")
file(GLOB src "*.cpp")

add_executable(${PROJECT_NAME} main.cpp render.cpp)
target_link_libraries(${PROJECT_NAME}  ${PCL_LIBRARIES} ${Boost_LIBRARIES})

#add_custom_target(RunTests COMMAND $<TARGET_FILE:${PROJECT_NAME}_Test> "--log_level=message" DEPENDS ${PROJECT_NAME})
