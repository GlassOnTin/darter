/*
 * Dumps the exact byte layout of the Betaflight SITL bridge structs as the
 * compiler lays them out. The darter-core wire-format test compares its
 * manual little-endian packing against these golden bytes.
 *
 * Struct definitions are included from the real upstream header:
 *   Betaflight src/platform/SIMULATOR/target/SITL/target.h
 * (GPLv3; used for verification, not linked).
 *
 * Build:
 *   gcc -I<path-to-betaflight>/src/platform/SIMULATOR/target/SITL \
 *       -o /tmp/wire_dump tools/wire_dump.c
 *   /tmp/wire_dump > tests/data/{fdm,rc,servo}_golden.bin
 */
#include <stdio.h>
#include "target.h"

int main(void) {
    fdm_packet f;
    f.timestamp = 1.5;
    for (int i = 0; i < 3; i++) f.imu_angular_velocity_rpy[i] = 0.1 + 1.0 * i;
    for (int i = 0; i < 3; i++) f.imu_linear_acceleration_xyz[i] = 4.0 + i;
    for (int i = 0; i < 4; i++) f.imu_orientation_quat[i] = 7.0 + i;
    for (int i = 0; i < 3; i++) f.velocity_xyz[i] = 11.0 + i;
    for (int i = 0; i < 3; i++) f.position_xyz[i] = 14.0 + i;
    f.pressure = 17.0;

    rc_packet r;
    r.timestamp = 2.5;
    for (int i = 0; i < 16; i++) r.channels[i] = 1000 + 10 * i;

    servo_packet s;
    for (int i = 0; i < 4; i++) s.motor_speed[i] = 0.1f + 0.1f * i;

    fwrite(&f, 1, sizeof f, stdout);
    fwrite(&r, 1, sizeof r, stdout);
    fwrite(&s, 1, sizeof s, stdout);
    fprintf(stderr, "sizeof fdm=%zu rc=%zu servo=%zu\n", sizeof f, sizeof r, sizeof s);
    return 0;
}