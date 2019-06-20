//
// Created by Franz Pucher on 2019-05-23.
//

#ifndef BACHELORTEST_CONSTANTS_H
#define BACHELORTEST_CONSTANTS_H

// Bits used in the overrides image bytes
enum OverrideFlags
{
    OF_RIVER_MARSH = 0x10,
    OF_INLAND = 0x20,
    OF_WATER_BASIN = 0x40,
    OF_PATH = 0x01,
    OF_VISITED = 0x02,
};

// Some constants
enum {
    IMAGE_DIM = 2048, // Width and height of the elevation and overrides image

    ROVER_X = 159,
    ROVER_Y = 1520,
    BACHELOR_X = 1303,
    BACHELOR_Y = 85,
    WEDDING_X = 1577,
    WEDDING_Y = 1294
};

#endif //BACHELORTEST_CONSTANTS_H
