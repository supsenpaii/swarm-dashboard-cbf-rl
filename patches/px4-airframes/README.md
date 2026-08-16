# PX4 Sparrow airframe

`4020_gz_sparrow_gimbal` is installed into PX4's
`ROMFS/px4fmu_common/init.d-posix/airframes/` and listed in that directory's
`CMakeLists.txt`. Rebuild `px4_sitl_default` after installing it.

The airframe uses the native Sparrow rotor geometry and documented AutoTune
gains. Its `MPC_THR_HOVER=0.66` is provisional for the 3.12 kg Sparrow plus
the dashboard's 0.10 kg lidar payload; validate hover before active flight.
