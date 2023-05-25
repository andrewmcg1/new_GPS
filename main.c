#include "naza_gps.h"

int main()
{
    int yay = gps_init();
    while(1)
        yay = gps_getData();

    return 0;
}
