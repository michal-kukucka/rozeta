#pragma once
#ifdef __cplusplus
extern "C" {
#endif
typedef struct RozetaPose2D { double x; double y; double heading; } RozetaPose2D;
typedef struct RozetaObstacleInfo { int obstacleAhead; int obstacleLeft; int obstacleRight; double nearestDistance; } RozetaObstacleInfo;
const char* rozeta_version(void);
#ifdef __cplusplus
}
#endif
