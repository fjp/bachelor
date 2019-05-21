//
// Created by Franz Pucher on 2019-05-22.
//

#ifndef BACHELORTEST_UTILITIES_H
#define BACHELORTEST_UTILITIES_H


#include "visualizer.h"
#include <fstream>
#include <string>
#include <vector>
#include <exception>




#include "planner.h"
#include "structs.h"
#include "audi_rover.h"

#include "location.h"

using namespace planner;

#ifdef _MSC_VER
static const char* PATH_SEP = "\\";
#else
static const char* PATH_SEP = "/";
#endif

// Bits used in the overrides image bytes
enum OverrideFlags
{
    OF_RIVER_MARSH = 0x10,
    OF_INLAND = 0x20,
    OF_WATER_BASIN = 0x40,
    OF_PATH = 0x01,
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

std::ifstream::pos_type fileSize(const std::string& filename)
{
    std::ifstream in(filename, std::ifstream::ate | std::ifstream::binary);
    if (!in.good())
    {
        throw std::exception();
    }
    return in.tellg();
}

std::vector<uint8_t> loadFile(const std::string& filename, size_t expectedFileSize)
{
    size_t fsize = fileSize(filename);
    if (fsize != expectedFileSize)
    {
        throw std::exception();
    }
    std::vector<uint8_t> data(fsize);
    std::ifstream ifile(filename, std::ifstream::binary);
    if (!ifile.good())
    {
        throw std::exception();
    }
    ifile.read((char*)&data[0], fsize);
    return data;
}

bool donut(int x, int y, int x1, int y1)
{
    int dx = x - x1;
    int dy = y - y1;
    int r2 = dx * dx + dy * dy;
    return r2 >= 150 && r2 <= 400;
}


bool path(int x, int y, uint8_t* overrides)
{

    for (int i = -3; i < 3; ++i)
    {
        int dx = x - i;
        for (int j = -3; j < 3; ++j)
        {
            int dy = y - j;
            if ((dx >= 0 && dx < IMAGE_DIM) && (dy >= 0 && dy < IMAGE_DIM) && (overrides[dy * IMAGE_DIM + dx] & (OF_PATH)))
            {
                return true;
            }
        }
    }

    return false;
}

#endif //BACHELORTEST_UTILITIES_H
