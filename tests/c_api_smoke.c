#include <rozeta/c_api.h>

#include <math.h>
#include <stdio.h>
#include <string.h>

static int near_double(double actual, double expected) {
    return fabs(actual - expected) < 1e-9;
}

int main(void) {
    const char* version = rozeta_version();
    if (version == NULL || strcmp(version, "0.1.0") != 0) {
        fprintf(stderr, "unexpected version: %s\n", version == NULL ? "(null)" : version);
        return 1;
    }

    if (!near_double(rozeta_normalize_angle(4.0), -2.2831853071795862)) {
        fprintf(stderr, "angle normalization failed\n");
        return 1;
    }

    if (!near_double(rozeta_distance_2d(0.0, 0.0, 3.0, 4.0), 5.0)) {
        fprintf(stderr, "distance calculation failed\n");
        return 1;
    }

    RozetaLidarScanPoint scan[] = {
        {0.0, 2.0, 1},
        {10.0, 0.7, 1},
        {-50.0, 0.8, 1},
        {60.0, 0.9, 1},
        {180.0, 0.2, 0},
    };
    RozetaObstacleInfo info = rozeta_obstacles_from_lidar(scan, 5, 1.0);
    if (!info.obstacleAhead || !info.obstacleLeft || !info.obstacleRight ||
        !near_double(info.nearestDistance, 0.7)) {
        fprintf(stderr, "obstacle sector calculation failed\n");
        return 1;
    }

    RozetaObstacleInfo empty = rozeta_obstacles_from_lidar(NULL, 0, 1.0);
    if (empty.obstacleAhead || empty.obstacleLeft || empty.obstacleRight ||
        !near_double(empty.nearestDistance, 0.0)) {
        fprintf(stderr, "empty obstacle calculation failed\n");
        return 1;
    }

    return 0;
}
