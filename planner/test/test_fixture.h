//
// Created by Franz Pucher on 2019-05-22.
//

#ifndef BACHELORTEST_TEST_FIXTURE_H
#define BACHELORTEST_TEST_FIXTURE_H

#include "visualizer.h"
#include "utilities.h"

#include <stdio.h>  /* defines FILENAME_MAX */


// The fixture for testing class Foo.
class cPlannerTest : public ::testing::Test {
protected:
    // You can remove any or all of the following functions if its body
    // is empty.

    cPlannerTest() {
        // You can do set-up work for each test here.
    }

    ~cPlannerTest() override {
        // You can do clean-up work that doesn't throw exceptions here.
    }

    // If the constructor and destructor are not enough for setting up
    // and cleaning up each test, you can define the following methods:

    void SetUp() override {
        // Code here will be called immediately after the constructor (right
        // before each test).
        //std::cout << "SetUp Tests" << std::endl;


#ifdef WINDOWS
        #include <direct.h>
    #define GetCurrentDir _getcwd
#else
#include <unistd.h>
#define GetCurrentDir getcwd
#endif

        char cCurrentPath[FILENAME_MAX];

        if (!GetCurrentDir(cCurrentPath, sizeof(cCurrentPath)))
        {
            //return errno;
        }

        cCurrentPath[sizeof(cCurrentPath) - 1] = '\0'; /* not really required */

        //printf ("The current working directory is %s\n", cCurrentPath);

        /// Prepare to read elevation and overrides data
        const size_t expectedFileSize = IMAGE_DIM * IMAGE_DIM;
        // Address assets relative to application location
        std::string anchor = std::string(".") + PATH_SEP;
        std::string pname = cCurrentPath;
        auto lastpos = pname.find_last_of("/\\");
        if (lastpos != std::string::npos) {
            anchor = pname.substr(0, lastpos);
        }
        for (int i = 0; i < 2; ++i) {
            auto lastpos = anchor.find_last_of("/\\");
            if (lastpos != std::string::npos) {
                anchor = anchor.substr(0, lastpos);
            }
        }
        anchor += PATH_SEP;
        m_oElevation = loadFile(anchor + "assets" + PATH_SEP + "elevation.data", expectedFileSize);
        m_oOverrides = loadFile(anchor + "assets" + PATH_SEP + "overrides.data", expectedFileSize);
        m_oImage = std::ofstream("pic.bmp", std::ofstream::binary);
    }

    void TearDown() override {
        // Code here will be called immediately after each test (right
        // before the destructor).
        visualizer::writeBMP(
                m_oImage,
                &m_oElevation[0],
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

                    if (visualizer::path(x, y, &m_oOverrides[0]))
                    {
                        return uint8_t(visualizer::IPV_PATH);
                    }

                    // Signifies water
                    if ((m_oOverrides[y * IMAGE_DIM + x] & (OF_WATER_BASIN | OF_RIVER_MARSH)) ||
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
        m_oImage.flush();
#if __APPLE__
        auto res = system("open pic.bmp");
        (void)res;
#endif

    }

    // Objects declared here can be used by all tests in the test case for Foo.
    std::vector<uint8_t> m_oElevation;
    std::vector<uint8_t> m_oOverrides;

    std::ofstream m_oImage;
};

#endif //BACHELORTEST_TEST_FIXTURE_H
