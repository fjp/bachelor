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

    auto poAudiRover = std::make_shared<cAudiRover>(cAudiRover(&elevation[0], &overrides[0], IMAGE_DIM, IMAGE_DIM));


    std::vector<tLocation> asLocation;
    asLocation.push_back(tLocation{ROVER_X, ROVER_Y});
    asLocation.push_back(tLocation{BACHELOR_X, BACHELOR_Y});
    asLocation.push_back(tLocation{WEDDING_X, WEDDING_Y});

    poAudiRover->SetStart(tLocation{ROVER_X, ROVER_Y});
    poAudiRover->SetGoal(tLocation{BACHELOR_X, BACHELOR_Y});


    poAudiRover->Summon(1, 1, ASTAR);


    poAudiRover->SetStart({BACHELOR_X, BACHELOR_Y});
    poAudiRover->SetGoal({WEDDING_X, WEDDING_Y});


    poAudiRover->Summon(1, 1, ASTAR);


    /// Report the total planning time

    double fIslandSeconds = poAudiRover->TotalTime();
    std::cout << "\nTravelling will take " << fIslandSeconds << " island seconds ("
              << fIslandSeconds/60.f << " island minutes or " << fIslandSeconds/60.f/60.f << " island hours) on the fastest path. " << std::endl;

    //////////// Output the found path /////////////////////
    visualizer::write("pic_final.bmp", &elevation[0], &overrides[0], asLocation, IMAGE_DIM);
    /*
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

            if (visualizer::visited(x, y, &overrides[0]))
            {
                return uint8_t(visualizer::IPV_VISITED);
            }

            // Signifies normal ground color
            if (elevation < visualizer::IPV_ELEVATION_BEGIN)
            {
                elevation = visualizer::IPV_ELEVATION_BEGIN;
            }
            return elevation;
    });
    of.flush();
     */
#if __APPLE__
    auto res = system("open pic.bmp");
    (void)res;
#endif

    return 0;
}

