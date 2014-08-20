rock_find_pkgconfig(Eigen3 eigen3 REQUIRED)

rock_library(imu_imar
    SOURCES Imar.cpp
    HEADERS Imar.hpp)

rock_executable(imu_imar_bin Main.cpp
    DEPS imu_imar)

