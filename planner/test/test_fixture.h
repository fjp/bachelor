//
// Created by Franz Pucher on 2019-05-22.
//

#ifndef BACHELORTEST_TEST_FIXTURE_H
#define BACHELORTEST_TEST_FIXTURE_H

#include "visualizer.h"
#include "utilities.h"

#include <stdio.h>  /* defines FILENAME_MAX */


///\brief The fixture for testing class cPlannerTest.
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



        /// Create Audi rover
        m_poAudiRover = std::make_shared<cAudiRover>(&m_oElevation[0], &m_oOverrides[0], IMAGE_DIM, IMAGE_DIM);


        /// Bachelor calls Audi rover
        m_poAudiRover->SetStart(tLocation{ROVER_X, ROVER_Y});
        m_poAudiRover->SetGoal(tLocation{BACHELOR_X, BACHELOR_Y});


    }

    void TearDown() override {
        // Code here will be called immediately after each test (right
        // before the destructor).
        std::vector<tLocation> asLocation;
        asLocation.push_back(tLocation{ROVER_X, ROVER_Y});
        asLocation.push_back(tLocation{BACHELOR_X, BACHELOR_Y});
        asLocation.push_back(tLocation{WEDDING_X, WEDDING_Y});
        visualizer::write("test_pic2.bmp", &m_oElevation[0], &m_oOverrides[0], asLocation, IMAGE_DIM);


    }

    // Objects declared here can be used by all tests in the test case for Foo.
    std::vector<uint8_t> m_oElevation;
    std::vector<uint8_t> m_oOverrides;
    std::shared_ptr<cAudiRover> m_poAudiRover;
};

#endif //BACHELORTEST_TEST_FIXTURE_H
