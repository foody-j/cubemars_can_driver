cmake_minimum_required(VERSION 3.8)
project(can_test)

set(CMAKE_CXX_STANDARD 17)

# 소스 파일 위치 지정
add_executable(can_test
    ${CMAKE_SOURCE_DIR}/src/main_test.cpp
)

# 헤더 파일 디렉터리 추가
target_include_directories(can_test PRIVATE
    ${CMAKE_SOURCE_DIR}/include
)

target_link_libraries(can_test pthread)
