IF (CMAKE_BUILD_TYPE MATCHES "Debug")
    MESSAGE("Debug -g -O0 -rdynamic")
    SET(CMAKE_CXX_FLAGS_DEBUG "-g -O0")# -g show more info for debug
ELSE ()
    MESSAGE("Release -O3 -s")
    SET(CMAKE_CXX_FLAGS_RELEASE "-O3 -s -Wall")#-Wall show all the warning
ENDIF ()

FILE(GLOB_RECURSE SRCs "*.c*" "*.h*")
find_package(OpenCV REQUIRED)

#Generate the lib and executable file
#ADD_LIBRARY(${PROJECT_NAME} ${SRCs})
ADD_EXECUTABLE(${PROJECT_NAME} ${SRCs})
TARGET_LINK_LIBRARIES(${PROJECT_NAME} PUBLIC pthread dl ${OpenCV_LIBS})
message(STATUS "The opencv libs: ${OpenCV_LIBS}")
