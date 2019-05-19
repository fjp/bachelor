#include "visualizer.h"
#include <fstream>
#include <string>
#include <vector>
#include <exception>




#include "planner.h"
#include "structs.h"
#include "priority_queue.h"
#include "audi_rover.h"

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

    for (int i = -5; i < 5; ++i)
    {
        int dx = x - i;
        for (int j = -5; j < 5; ++j)
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

int main(int argc, char** argv)
{
    const size_t expectedFileSize = IMAGE_DIM * IMAGE_DIM;
    // Address assets relative to application location
    std::string anchor = std::string(".") + PATH_SEP;
    std::string pname = argv[0];
    auto lastpos = pname.find_last_of("/\\");
    if (lastpos != std::string::npos)
    {
        anchor = pname.substr(0, lastpos) + PATH_SEP;
    }
    auto elevation = loadFile(anchor + "assets" + PATH_SEP + "elevation.data", expectedFileSize);
    auto overrides = loadFile(anchor + "assets" + PATH_SEP + "overrides.data", expectedFileSize);
    std::ofstream of("pic.bmp", std::ofstream::binary);


    ////////////////////////////

    //cAudiRover<const uint8_t* > oAudiRover(&elevation[0], &overrides[0]);
    cAudiRover oAudiRover(&elevation[0], &overrides[0], IMAGE_DIM, IMAGE_DIM);
    oAudiRover.SetStart(ROVER_X, ROVER_Y);
    oAudiRover.SetGoal(BACHELOR_X, BACHELOR_Y);


    oAudiRover.Summon();



    ////////////////////////////

    visualizer::writeBMP(
        of,
        &elevation[0],
        IMAGE_DIM,
        IMAGE_DIM,
        [&] (size_t x, size_t y, uint8_t elevation) {
        
            // Marks interesting positions on the map
            if (donut(x, y, ROVER_X, ROVER_Y) ||
                donut(x, y, BACHELOR_X, BACHELOR_Y) ||
                donut(x, y, WEDDING_X, WEDDING_Y))
            {
                return uint8_t(visualizer::IPV_PATH);
            }

            if (path(x, y, &overrides[0]))
            {
                return uint8_t(visualizer::IPV_PATH);
            }
            
            // Signifies water
            if ((overrides[y * IMAGE_DIM + x] & (OF_WATER_BASIN | OF_RIVER_MARSH)) ||
                elevation == 0)
            {
                return uint8_t(visualizer::IPV_WATER);
            }
            
            // Signifies normal ground color
            if (elevation < visualizer::IPV_ELEVATION_BEGIN)
            {
                elevation = visualizer::IPV_ELEVATION_BEGIN;
            }
            return elevation;
    });
    of.flush();
#if __APPLE__
    auto res = system("open pic.bmp");
    (void)res;
#endif
    return 0;
}

