#include <rozeta/c_api.h>

#include <stdio.h>

int main(void) {
    RozetaLidarScanPoint scan[] = {
        {10.0, 0.75, 1},
        {-40.0, 0.80, 1},
    };
    RozetaObstacleInfo info = rozeta_obstacles_from_lidar(scan, 2, 1.0);
    printf("rozeta %s angle=%.3f distance=%.3f ahead=%d left=%d right=%d nearest=%.2f\n",
           rozeta_version(),
           rozeta_normalize_angle(4.0),
           rozeta_distance_2d(0.0, 0.0, 3.0, 4.0),
           info.obstacleAhead,
           info.obstacleLeft,
           info.obstacleRight,
           info.nearestDistance);
    return 0;
}
