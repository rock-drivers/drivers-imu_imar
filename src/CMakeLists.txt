rock_library(imar
    SOURCES imar.cpp
    HEADERS imar.hpp)

rock_executable(imar_bin Main.cpp
    DEPS imar)

