//
// Created by Franz Pucher on 2019-05-21.
//

#include <gtest/gtest.h>
#include <iostream>
#include <location.h>

#include "test_fixture.h"



TEST_F(cPlannerTest, simple_map)
{
    InitSimpleMap();

    int nStartX = 30; int nStartY = 30;
    int nGoalX = 300; int nGoalY = 300;
    CreateRover(tLocation{nStartX, nStartY}, tLocation{nGoalX, nGoalY});

    m_poAudiRover->Summon(1);
    float fTimeAStar = m_poAudiRover->TotalTime();
    m_poAudiRover->ResetTime();
    //m_poAudiRover->Summon(1, 1, ASTAR_OPT);
    //float fTimeAStarOpt = m_poAudiRover->TotalTime();
    m_poAudiRover->Summon(1, 1, ASTAR_CK);
    float fTimeAStarCk = m_poAudiRover->TotalTime();

    std::vector<tLocation> asLocation;
    asLocation.push_back(tLocation{nStartX, nStartY});
    asLocation.push_back(tLocation{nGoalX, nGoalY});
    visualizer::write("simple_map.bmp", &m_oElevation[0], &m_oOverrides[0], asLocation, m_nImageDim);

    m_poAudiRover->countPlanner();
}



TEST_F(cPlannerTest, island_map)
{
    InitIslandMap();

    CreateRover(tLocation{ROVER_X, ROVER_Y}, tLocation{BACHELOR_X, BACHELOR_Y});


    m_poAudiRover->SetStart({BACHELOR_X, BACHELOR_Y});
    m_poAudiRover->SetGoal({WEDDING_X, WEDDING_Y});

    m_poAudiRover->Summon(1);

    std::vector<tLocation> asLocation;
    asLocation.push_back(tLocation{ROVER_X, ROVER_Y});
    asLocation.push_back(tLocation{BACHELOR_X, BACHELOR_Y});
    asLocation.push_back(tLocation{WEDDING_X, WEDDING_Y});

    visualizer::write("island.bmp", &m_oElevation[0], &m_oOverrides[0], asLocation, IMAGE_DIM);

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