#include <rozeta/c_api.h>

#include <stddef.h>
#include <stdio.h>

int main(void) {
    RozetaLidarScanPoint scan[] = {
        {10.0, 0.75, 1},
        {-40.0, 0.80, 1},
    };
    RozetaObstacleInfo info = rozeta_obstacles_from_lidar(scan, 2, 1.0);

    /* One 2x2 rgb24 frame against itself: nothing is new, so nothing trips. */
    unsigned char frame[2 * 2 * 3];
    RozetaRgbObstacleConfig obstacle_config = rozeta_rgb_obstacle_default_config();
    void* tracker = rozeta_rgb_obstacle_tracker_create(obstacle_config);
    RozetaRgbObstacleResult obstacle = {0};
    if (tracker != NULL) {
        size_t index;
        for (index = 0; index < sizeof(frame); ++index) {
            frame[index] = 200;
        }
        rozeta_rgb_obstacle_tracker_update_ref(tracker, frame, frame, 2, 2);
        obstacle = rozeta_rgb_obstacle_tracker_result(tracker);
        rozeta_rgb_obstacle_tracker_destroy(tracker);
    }
    printf("rozeta %s angle=%.3f distance=%.3f ahead=%d left=%d right=%d nearest=%.2f\n",
           rozeta_version(),
           rozeta_normalize_angle(4.0),
           rozeta_distance_2d(0.0, 0.0, 3.0, 4.0),
           info.obstacleAhead,
           info.obstacleLeft,
           info.obstacleRight,
           info.nearestDistance);
    printf("rgb obstacle state=%d diff=%.3f source=%s\n",
           obstacle.state,
           obstacle.diff_coverage,
           obstacle.source);
    return 0;
}
