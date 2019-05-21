//
// Created by Franz Pucher on 2019-05-21.
//

#include <gtest/gtest.h>
#include <iostream>
#include <location.h>

#include "test_fixture.h"

TEST_F(cPlannerTest, planner)
{

    /// Create Audi rover
    cAudiRover oAudiRover(&m_oElevation[0], &m_oOverrides[0], IMAGE_DIM, IMAGE_DIM);

    /// Bachelor calls Audi rover
    oAudiRover.SetStart(tLocation{ROVER_X, ROVER_Y});
    oAudiRover.SetGoal(tLocation{BACHELOR_X, BACHELOR_Y});
    oAudiRover.Summon();


    /// Drives Bachelor to wedding
    oAudiRover.SetStart({BACHELOR_X, BACHELOR_Y});
    oAudiRover.SetGoal({WEDDING_X, WEDDING_Y});
    oAudiRover.Summon();

    // TODO assert if path does not match

    // TODO output time it took the rover to get to its goal locations (unit island seconds)

}


TEST_F(cPlannerTest, aaaa)
{




    planner::tLocation sLocation{1, 2};
    std::cout << "Tests work" << std::endl;

}
