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

    cPlannerTest() // : m_poAudiRover(std::make_shared<cAudiRover>(nullptr, nullptr, 0, 0))
    {
        // You can do set-up work for each test here.
        m_nImageDim = 330;

    }

    ~cPlannerTest() override {
        // You can do clean-up work that doesn't throw exceptions here.
    }

    // If the constructor and destructor are not enough for setting up
    // and cleaning up each test, you can define the following methods:

    void ReadData(std::string i_strElevation, std::string i_strOverrides)
    {
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
        const size_t expectedFileSize = m_nImageDim * m_nImageDim;
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
        m_oElevation = loadFile(anchor + "assets" + PATH_SEP + i_strElevation, expectedFileSize);
        m_oOverrides = loadFile(anchor + "assets" + PATH_SEP + i_strOverrides, expectedFileSize);
    };

    void InitIslandMap()
    {
        m_nImageDim = IMAGE_DIM;

        ReadData("elevation.data", "overrides.data");
    }

    void SetUp() override {
        // Code here will be called immediately after the constructor (right
        // before each test).

        //InitIslandMap();
    }

    void CreateSimpleMap()
    {
        /// Prepare to read elevation and overrides data
        m_nImageDim = 330;
        const size_t expectedFileSize = m_nImageDim * m_nImageDim;

        std::vector<uint8_t> data_elevation(expectedFileSize, 1);
        writeFile("../../../assets/elevation_simple_map.data", data_elevation, expectedFileSize);

        std::vector<uint8_t> dataOverrides(expectedFileSize, 0);
        writeFile("../../../assets/overrides_simple_map.data", dataOverrides, expectedFileSize);
    }

    void InitSimpleMap()
    {
        CreateSimpleMap();

        m_nImageDim = 330;

        ReadData("elevation_simple_map.data", "overrides_simple_map.data");
    }


    void CreateSimpleMapWithWater()
    {
        /// Prepare to read elevation and overrides data
        m_nImageDim = 330;
        const size_t expectedFileSize = m_nImageDim * m_nImageDim;

        /// Create water
        std::vector<uint8_t> data_elevation(expectedFileSize, 1);
        for (int nY = 100; nY <= 110; ++nY)
        {
            for (int x = 50; x <= 100; ++x) {
                data_elevation[nY * m_nImageDim + x] = 0;
            }
        }

        for (int nY = 50; nY <= 100; ++nY)
        {
            for (int nX = 100; nX <= 110; ++nX) {
                data_elevation[nY * m_nImageDim + nX] = 0;
            }
        }
        writeFile("../../../assets/elevation_simple_map_with_water.data", data_elevation, expectedFileSize);

        std::vector<uint8_t> dataOverrides(expectedFileSize, 0);
        writeFile("../../../assets/overrides_simple_map_with_water.data", dataOverrides, expectedFileSize);
    }

    void InitSimpleMapWithWater()
    {
        CreateSimpleMapWithWater();

        m_nImageDim = 330;

        ReadData("elevation_simple_map_with_water.data", "overrides_simple_map_with_water.data");
    }

    void CreateSimpleMapWithElevation()
    {
        /// Prepare to read elevation and overrides data
        m_nImageDim = 4;
        const size_t expectedFileSize = m_nImageDim * m_nImageDim;

        /// Create mountain
        std::vector<uint8_t> data_elevation(expectedFileSize, 1);
        for (int nY = 0; nY < m_nImageDim; ++nY)
        {
            for (int nX = 0; nX < m_nImageDim; ++nX) {
                if (m_nImageDim - 2 <= nX + nY && nX + nY <= m_nImageDim) {
                    data_elevation[nY * m_nImageDim + nX] = 255;
                }
            }
        }

        writeFile("../../../assets/elevation_simple_map_with_elevation.data", data_elevation, expectedFileSize);

        std::vector<uint8_t> dataOverrides(expectedFileSize, 0);
        writeFile("../../../assets/overrides_simple_map_with_elevation.data", dataOverrides, expectedFileSize);

    }

    void InitSimpleMapWithElevation()
    {
        CreateSimpleMapWithElevation();

        ReadData("elevation_simple_map_with_elevation.data", "overrides_simple_map_with_elevation.data");
    }


    void CreateRover(const tLocation& i_sStart, const tLocation& i_sGoal)
    {
        /// Create Audi rover
        m_poAudiRover = std::make_shared<cAudiRover>(&m_oElevation[0], &m_oOverrides[0], m_nImageDim, m_nImageDim);

        /// Bachelor calls Audi rover
        m_poAudiRover->SetStart(i_sStart);
        m_poAudiRover->SetGoal(i_sGoal);

        //return m_poAudiRover;
    }

    void TearDown() override {
        // Code here will be called immediately after each test (right before the destructor).

    }

    // Objects declared here can be used by all tests in the test case for Foo.
    std::vector<uint8_t> m_oElevation;
    std::vector<uint8_t> m_oOverrides;
    std::shared_ptr<cAudiRover> m_poAudiRover;

    int m_nImageDim;
};

#endif //BACHELORTEST_TEST_FIXTURE_H
