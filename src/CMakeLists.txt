rock_library(stim300
    SOURCES Dummy.cpp
    HEADERS Dummy.hpp)

rock_executable(stim300_bin Main.cpp
    DEPS stim300)

