rock_find_pkgconfig(Eigen3 eigen3 REQUIRED)

rock_library(stim300
    SOURCES Stim300Base.cpp
            Stim300RevD.cpp
            Stim300RevB.cpp
    HEADERS Stim300Base.hpp
            Stim300RevD.hpp
            Stim300RevB.hpp
    DEPS_PKGCONFIG iodrivers_base)

rock_executable(stim300_bin main.cpp
    DEPS stim300
    DEPS_PKGCONFIG iodrivers_base)

