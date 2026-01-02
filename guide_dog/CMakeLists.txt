cmake_minimum_required(VERSION 3.5)
project(guide_dog_example)

set(CMAKE_CXX_STANDARD 17)

add_executable(guide_dog guide_dog.cpp)

# If you want to enable integration with the unitree SDK, add definitions
# and link the SDK libraries here. For example:
# target_compile_definitions(guide_dog PRIVATE USE_UNITREE_SDK)
# target_include_directories(guide_dog PRIVATE ${CMAKE_SOURCE_DIR}/../include)
# target_link_libraries(guide_dog PRIVATE unitree_common unitree_robot)

add_executable(udp_to_sport udp_to_sport.cpp)
target_link_libraries(udp_to_sport PRIVATE unitree_common unitree_robot)
