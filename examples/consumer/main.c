#include <rozeta/c_api.h>

#include <stdio.h>

int main(void) {
    printf("%s %.1f\n", rozeta_version(), rozeta_distance_2d(0.0, 0.0, 3.0, 4.0));
    return 0;
}
