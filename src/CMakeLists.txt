rock_library(imar
    SOURCES Dummy.cpp
    HEADERS Dummy.hpp)

rock_executable(imar_bin Main.cpp
    DEPS imar)

