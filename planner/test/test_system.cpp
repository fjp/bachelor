//
// Created by Franz Pucher on 2019-05-21.
//

#include <gtest/gtest.h>
#include <iostream>
#include <location.h>

#include "test_fixture.h"

TEST_F(cPlannerTest, simple_map)
{
    /// Prepare to read elevation and overrides data
    size_t nImageDim = 600;
    const size_t expectedFileSize = nImageDim * nImageDim;

    std::vector<uint8_t> data_elevation(expectedFileSize, 1);
    writeFile("../../../assets/test_elevation.data", data_elevation, expectedFileSize);
    std::vector<uint8_t> dataOverrides(expectedFileSize, 0);
    writeFile("../../../assets/test_overrides.data", dataOverrides, expectedFileSize);

    auto oElevation = loadFile("../../../assets/test_elevation.data", expectedFileSize);
    auto oOverrides = loadFile("../../../assets/test_overrides.data", expectedFileSize);


    /// Create Audi rover
    //cAudiRover oAudiRover(&oElevation[0], &oOverrides[0], nImageDim, nImageDim);
    auto poAudiRover = std::make_shared<cAudiRover>(&oElevation[0], &oOverrides[0], nImageDim, nImageDim);

    /// Bachelor calls Audi rover
    int nStartX = 20; int nStartY = 20;
    int nGoalX = 100; int nGoalY = 100;
    poAudiRover->SetStart(tLocation{nStartX, nStartY});
    poAudiRover->SetGoal(tLocation{nGoalX, nGoalY});
    poAudiRover->Summon(1);

    std::vector<tLocation> asLocation;
    asLocation.push_back(tLocation{nStartX, nStartY});
    asLocation.push_back(tLocation{nGoalX, nGoalY});
    visualizer::write("test_pic.bmp", &oElevation[0], &oOverrides[0], asLocation, nImageDim);

    poAudiRover->countPlanner();

}

/*
TEST_F(cPlannerTest, DISABLED_step_cost)
{

    uint8_t nStepSize = 1;
    uint8_t nVelocity = 1; /// 1 cell per island second

    m_poAudiRover->InitializePlanner(nStepSize, nVelocity);
    cPlanner *poPlanner = static_cast<cPlanner*>(m_poAudiRover->GetPlanner());


    tNode *sStart = new tNode(tLocation{BACHELOR_X, BACHELOR_Y});

    tAction sAction = { 1, 0, m_poAudiRover->CostStraight() };
    tNode *sGoal = poPlanner->Child(sStart, sAction);
    poPlanner->UpdateCost(sGoal);
    EXPECT_EQ(sGoal->g, sAction.fCost) << "Step cost is not equal to action cost";


    sAction = { 1, 0, m_poAudiRover->CostDiagonal() };
    sGoal = poPlanner->Child(sStart, sAction);
    poPlanner->UpdateCost(sGoal);
    EXPECT_EQ(sGoal->g, sAction.fCost) << "Step cost is not equal to action cost";


    /// Step size 2
    m_poAudiRover->InitializePlanner(2, nVelocity);
    poPlanner = static_cast<cPlanner*>(m_poAudiRover->GetPlanner());


    sAction = { 1, 0, m_poAudiRover->CostStraight() };
    sGoal = poPlanner->Child(sStart, sAction);
    poPlanner->UpdateCost(sGoal);
    EXPECT_EQ(sGoal->g, sAction.fCost) << "Step cost is not equal to action cost";


    sAction = { 1, 0, m_poAudiRover->CostDiagonal() };
    sGoal = poPlanner->Child(sStart, sAction);
    poPlanner->UpdateCost(sGoal);
    EXPECT_EQ(sGoal->g, sAction.fCost) << "Step cost is not equal to action cost";


    bool bGoalReached = poPlanner->GoalTest(sStart, sGoal);
    EXPECT_EQ(bGoalReached, false);


    // TODO assert if path does not match

    // TODO output time it took the rover to get to its goal locations (unit island seconds)

}
 */
/*
TEST_F(cPlannerTest, DISABLED_rover_to_bachelor)
{

    /// Create Audi rover
    auto poAudiRover = std::make_shared<cAudiRover>(cAudiRover(&m_oElevation[0], &m_oOverrides[0], IMAGE_DIM, IMAGE_DIM));

    /// Bachelor calls Audi rover
    poAudiRover->SetStart(tLocation{ROVER_X, ROVER_Y});
    poAudiRover->SetGoal(tLocation{BACHELOR_X, BACHELOR_Y});
    poAudiRover->Summon(2);
    

    // TODO assert if path does not match

    // TODO output time it took the rover to get to its goal locations (unit island seconds)

}
*/
/*
TEST_F(cPlannerTest, rover_to_bachelor_to_wedding)
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


TEST_F(cPlannerTest, heuristic)
{
    uint8_t nStepSize = 1;
    uint8_t nVelocity = 1; /// 1 cell per island second

    m_poAudiRover->InitializePlanner(nStepSize, nVelocity);
    cPlanner *poPlanner = static_cast<cPlanner*>(m_poAudiRover->GetPlanner());

    poPlanner->GenerateHeuristic();
}
*/