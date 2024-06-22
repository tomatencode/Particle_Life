cmake_minimum_required(VERSION 3.28)
project(ParticleLife)

include(FetchContent)

FetchContent_Declare(
  eigen
  GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
  GIT_TAG        3.4.0
)
FetchContent_MakeAvailable(eigen)

set(CMAKE_PREFIX_PATH /home/simon/Qt/6.7.2/gcc_64/)
find_package(Qt6 REQUIRED COMPONENTS Charts Core Gui)
qt_standard_project_setup()

qt_add_executable(ParticleLife
    main.cpp Display_Particle.hpp
)

set_target_properties(ParticleLife PROPERTIES
    WIN32_EXECUTABLE TRUE
    MACOSX_BUNDLE TRUE
)

set_target_properties(ParticleLife PROPERTIES
    WIN32_EXECUTABLE TRUE
    MACOSX_BUNDLE TRUE
)

set_property(TARGET ParticleLife PROPERTY CXX_STANDARD 20)
target_link_libraries(ParticleLife PUBLIC
eigen  
Qt::Charts
Qt::Core
Qt::Gui
)