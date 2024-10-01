cmake_minimum_required(VERSION 3.16 FATAL_ERROR)

project(JopenGL C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

# Limit configurations to those Jolt supports
set(CMAKE_CONFIGURATION_TYPES "Debug;Release;Distribution")

# OpenAL build options
set(ALSOFT_UTILS OFF CACHE BOOL "" FORCE)
set(ALSOFT_EXAMPLES OFF CACHE BOOL "" FORCE)
set(ALSOFT_TESTS OFF CACHE BOOL "" FORCE)

# When turning this option on, the Jolt will be compiled using doubles for positions. This allows for much bigger worlds.
set(DOUBLE_PRECISION ON)

include(FetchContent) # this makes managing dependencies simpler by automatically downloading them
FetchContent_Declare(
        glfw
        GIT_REPOSITORY https://github.com/glfw/glfw.git
        GIT_TAG 3.3.8
)
FetchContent_Declare(
        glew
        GIT_REPOSITORY https://github.com/Perlmint/glew-cmake.git
        GIT_TAG glew-cmake-2.2.0
)
FetchContent_Declare(
        glm
        GIT_REPOSITORY https://github.com/g-truc/glm.git
        GIT_TAG 0.9.9.8
)
FetchContent_Declare(
        JoltPhysics
        GIT_REPOSITORY https://github.com/jrouwe/JoltPhysics.git
        GIT_TAG v4.0.2
        SOURCE_SUBDIR Build
)
FetchContent_Declare(
        openal
        GIT_REPOSITORY https://github.com/kcat/openal-soft.git
        GIT_TAG 1.23.1
)
#FetchContent_Declare(assimp
#        GIT_REPOSITORY https://github.com/assimp/assimp.git
#        GIT_TAG master
#)
FetchContent_MakeAvailable(glfw glew glm JoltPhysics openal) # assimp
#
#set(ASSIMP_BUILD_TESTS OFF CACHE BOOL "" FORCE)
#set(ASSIMP_INJECT_DEBUG_POSTFIX OFF CACHE BOOL "" FORCE)
#set(ASSIMP_INSTALL OFF CACHE BOOL "" FORCE)

file(GLOB_RECURSE SOURCES "src/*.cpp" "src/*.c")

add_executable(${PROJECT_NAME} ${SOURCES})

# folders include as dependencies
target_include_directories(${PROJECT_NAME} PRIVATE
        ${CMAKE_SOURCE_DIR}/include
        ${JoltPhysics_SOURCE_DIR}
        ${glm_SOURCE_DIR}
        ${glew_SOURCE_DIR}/include
        ${openal_SOURCE_DIR}/include
        #${assimp_SOURCE_DIR}/include
        ${CMAKE_SOURCE_DIR}/include
        ${CMAKE_SOURCE_DIR}/libraries/include
)

# linking stage
target_link_libraries(${PROJECT_NAME} PRIVATE
        Jolt
        glfw
        glm
        libglew_static
        OpenAL::OpenAL
        #assimp
)

# Settings for Windows
if(MSVC)
    target_compile_options(${PROJECT_NAME} PRIVATE /W4 /WX-)
    target_compile_definitions(${PROJECT_NAME} PRIVATE
            WIN32_LEAN_AND_MEAN
            NOMINMAX
            GLEW_STATIC
    )
else()
    target_compile_definitions(${PROJECT_NAME} PRIVATE GLEW_STATIC)
endif()

# Copy assets to build directory
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory
        "${CMAKE_SOURCE_DIR}/assets"
        "$<TARGET_FILE_DIR:${PROJECT_NAME}>/assets"
)