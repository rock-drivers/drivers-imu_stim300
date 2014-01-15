rock_find_pkgconfig(Eigen3 eigen3 REQUIRED)

rock_library(stim300
    SOURCES stim300.cpp
    HEADERS stim300.hpp
    DEPS_PKGCONFIG iodrivers_base)

rock_executable(stim300_bin main.cpp
    DEPS stim300
    DEPS_PKGCONFIG iodrivers_base)

