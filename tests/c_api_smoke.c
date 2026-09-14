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

    /* Two ways between junctions A (1) and B (3): straight through 2, or a
     * detour through 5 and 6. Closing the straight way sends the route round. */
    {
        const double lat0 = 49.0845;
        const double lon0 = 17.3361;
        const double m_lat = 1.0 / 111194.9;
        const double m_lon = 1.0 / (111194.9 * cos(lat0 * 3.141592653589793 / 180.0));
        const double east[] = {-30.0, 0.0, 50.0, 100.0, 130.0, 0.0, 100.0};
        const double north[] = {0.0, 0.0, 0.0, 0.0, 0.0, 40.0, 40.0};
        double vlat[7];
        double vlon[7];
        const int from[] = {0, 1, 2, 3, 1, 5, 6};
        const int to[] = {1, 2, 3, 4, 5, 6, 3};
        int section[16];
        double out_lat[512];
        double out_lon[512];
        int closed_from[8];
        int closed_to[8];
        int index;
        void* graph;
        RozetaGraphSectionResult straight;
        RozetaGraphRouteResult open_route;
        RozetaGraphRouteResult detour;
        for (index = 0; index < 7; ++index) {
            vlat[index] = lat0 + north[index] * m_lat;
            vlon[index] = lon0 + east[index] * m_lon;
        }
        graph = rozeta_graph_create(vlat, vlon, 7, from, to, 7);
        if (graph == NULL) {
            fprintf(stderr, "graph creation failed\n");
            return 1;
        }
        straight = rozeta_graph_section_at(graph, vlat[2] + 2.0 * m_lat, vlon[2] + 20.0 * m_lon,
                                           10.0, section, 16);
        if (!straight.ok || straight.vertex_count != 3 || section[0] != 1 || section[2] != 3 ||
            fabs(straight.length_m - 100.0) > 0.5) {
            fprintf(stderr, "graph section failed: %s\n", straight.message);
            rozeta_graph_destroy(graph);
            return 1;
        }
        open_route = rozeta_graph_plan_route(graph, vlat[0], vlon[0], vlat[4], vlon[4],
                                             NULL, NULL, 0, 25.0, 0.0, out_lat, out_lon, 512);
        for (index = 1; index < straight.vertex_count; ++index) {
            closed_from[index - 1] = section[index - 1];
            closed_to[index - 1] = section[index];
        }
        detour = rozeta_graph_plan_route(graph, vlat[0], vlon[0], vlat[4], vlon[4],
                                         closed_from, closed_to, straight.vertex_count - 1,
                                         25.0, 2.0, out_lat, out_lon, 512);
        rozeta_graph_destroy(graph);
        if (!open_route.ok || fabs(open_route.distance_m - 160.0) > 0.5) {
            fprintf(stderr, "open route failed: %s\n", open_route.message);
            return 1;
        }
        if (!detour.ok || fabs(detour.distance_m - 240.0) > 0.5 || detour.point_count < 100) {
            fprintf(stderr, "closed-section route failed: %s\n", detour.message);
            return 1;
        }
    }
    if (rozeta_graph_create(NULL, NULL, 0, NULL, NULL, 0) != NULL) {
        fprintf(stderr, "an empty graph should not be created\n");
        return 1;
    }
    rozeta_graph_destroy(NULL);

    return 0;
}
