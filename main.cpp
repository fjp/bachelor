#include "utilities.h"
#include "visualizer.h"

#include <iostream>

int main(int argc, char** argv)
{
    /// Prepare to read elevation and overrides data
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



    //////////////// Create the Audi rover and use its summon feature ///////////////

    cAudiRover oAudiRover(&elevation[0], &overrides[0], IMAGE_DIM, IMAGE_DIM);


    oAudiRover.SetStart(tLocation{ROVER_X, ROVER_Y});
    oAudiRover.SetGoal(tLocation{BACHELOR_X, BACHELOR_Y});


    //oAudiRover.Summon(1);


    oAudiRover.SetStart({BACHELOR_X, BACHELOR_Y});
    oAudiRover.SetGoal({WEDDING_X, WEDDING_Y});


    oAudiRover.Summon(1);


    /// Report the total planning time

    float fIslandSeconds = oAudiRover.TotalTime();
    std::cout << "\nTravelling will take " << fIslandSeconds << " island seconds ("
              << fIslandSeconds/60.f << " island minutes or " << fIslandSeconds/60.f/60.f << " island hours) on the fastest path. " << std::endl;

    //////////// Output the found path /////////////////////
    std::ofstream of("pic.bmp", std::ofstream::binary);
    visualizer::writeBMP(
        of,
        &elevation[0],
        IMAGE_DIM,
        IMAGE_DIM,
        [&] (size_t x, size_t y, uint8_t elevation) {
        
            // Marks interesting positions on the map
            if (visualizer::donut(x, y, ROVER_X, ROVER_Y) ||
                visualizer::donut(x, y, BACHELOR_X, BACHELOR_Y) ||
                visualizer::donut(x, y, WEDDING_X, WEDDING_Y))
            {
                return uint8_t(visualizer::IPV_PATH);
            }

            if (visualizer::path(x, y, &overrides[0]))
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

