rock_find_pkgconfig(Eigen3 eigen3 REQUIRED)

rock_library(imar
    SOURCES imar.cpp
    HEADERS imar.hpp)

rock_executable(imar_bin Main.cpp
    DEPS imar)

