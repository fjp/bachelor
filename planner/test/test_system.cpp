//
// Created by Franz Pucher on 2019-05-21.
//

#include <gtest/gtest.h>
#include <iostream>
#include <location.h>

#include "test_fixture.h"
#include "result.h"


TEST_F(cPlannerTest, simple_map_with_water)
{
    InitSimpleMapWithWater();

    int nStartX = 30; int nStartY = 30;
    int nGoalX = 300; int nGoalY = 300;
    CreateRover(tLocation{ nStartX, nStartY }, tLocation{ nGoalX, nGoalY });

    /// Test cPlanner::AStar implementation
    tResult sResultAStar = m_poAudiRover->Summon(1, 1, "ASTAR");
    EXPECT_TRUE(sResultAStar.bFoundGoal) << "Goal not found.";
    EXPECT_TRUE(sResultAStar.bConsistentHeuristic) << "Heuristic not consistent.";
    double fTimeAStar = m_poAudiRover->TotalTime();
    m_poAudiRover->ResetTime();

    /// Test cPlannerWiki::AStar() implementation
    tResult sResultAStarWiki = m_poAudiRover->Summon(1, 1, "ASTAR_WIKI");
    EXPECT_TRUE(sResultAStarWiki.bFoundGoal) << "Goal not found.";
    EXPECT_TRUE(sResultAStarWiki.bConsistentHeuristic) << "Heuristic not consistent.";
    double fTimeAStarWiki = m_poAudiRover->TotalTime();
    m_poAudiRover->ResetTime();

    /// Test cPlannerRBG::AStar() implementation
    tResult sResultAStarRBG = m_poAudiRover->Summon(1, 1, "ASTAR_RBG");
    EXPECT_TRUE(sResultAStarRBG.bFoundGoal) << "Goal not found.";
    EXPECT_TRUE(sResultAStarRBG.bConsistentHeuristic) << "Heuristic not consistent.";
    double fTimeAStarRBG = m_poAudiRover->TotalTime();

    EXPECT_NEAR(fTimeAStar, fTimeAStarWiki, 1.0e-8);
    EXPECT_NEAR(fTimeAStarWiki, fTimeAStarRBG, 1.0e-8);

    /// Output the found paths between the defined locations
    std::vector<tLocation> asLocation;
    asLocation.push_back(tLocation{ nStartX, nStartY });
    asLocation.push_back(tLocation{ nGoalX, nGoalY });
    visualizer::write("simple_map_with_water.bmp", &m_oElevation[0], &m_oOverrides[0], asLocation, m_nImageDim);
}

TEST_F(cPlannerTest, simple_map_with_elevation)
{
    InitSimpleMapWithElevation();

    int nStartX = 0; int nStartY = 0;
    //int nGoalX = 300; int nGoalY = 30;
    int nGoalX = m_nImageDim - 1; int nGoalY = m_nImageDim - 1;
    CreateRover(tLocation{nStartX, nStartY}, tLocation{nGoalX, nGoalY});

    /// Test cPlannerWiki::AStar() implementation
    tResult sResult = m_poAudiRover->Summon(1, 1, "ASTAR");
    EXPECT_TRUE(sResult.bFoundGoal) << "Goal not found.";
    EXPECT_TRUE(sResult.bConsistentHeuristic) << "Heuristic not consistent.";
    double fTimeAStar = m_poAudiRover->TotalTime();
    m_poAudiRover->ResetTime();

    std::vector<tLocation> asLocation;
    asLocation.push_back(tLocation{nStartX, nStartY});
    asLocation.push_back(tLocation{nGoalX, nGoalY});
    visualizer::write("simple_map_with_elevation.bmp", &m_oElevation[0], &m_oOverrides[0], asLocation, m_nImageDim);

    m_poAudiRover->countPlanner();
}

TEST_F(cPlannerTest, island_map_astar)
{
    InitIslandMap();

    CreateRover(tLocation{ROVER_X, ROVER_Y}, tLocation{BACHELOR_X, BACHELOR_Y});

    /// Test cPlanner::AStar implementation moving from rover to bachelor
    m_poAudiRover->SetStart(tLocation{ROVER_X, ROVER_Y});
    m_poAudiRover->SetGoal(tLocation{BACHELOR_X, BACHELOR_Y});
    tResult sResultR2B = m_poAudiRover->Summon(1, 1, "ASTAR");
    EXPECT_TRUE(sResultR2B.bFoundGoal);
    EXPECT_TRUE(sResultR2B.bConsistentHeuristic);

    /// Test cPlanner::AStar implementation moving from bachelor to wedding
    m_poAudiRover->SetStart({BACHELOR_X, BACHELOR_Y});
    m_poAudiRover->SetGoal({WEDDING_X, WEDDING_Y});
    tResult sResultB2W = m_poAudiRover->Summon(1, 1, "ASTAR");
    EXPECT_TRUE(sResultB2W.bFoundGoal);
    EXPECT_TRUE(sResultB2W.bConsistentHeuristic);


    //EXPECT_NEAR(fTimeAStar, fTimeAStarWiki, 1.0e-8);
    //EXPECT_NEAR(fTimeAStarWiki, fTimeAStarRBG, 1.0e-8);

    std::vector<tLocation> asLocation;
    asLocation.push_back(tLocation{ROVER_X, ROVER_Y});
    asLocation.push_back(tLocation{BACHELOR_X, BACHELOR_Y});
    asLocation.push_back(tLocation{WEDDING_X, WEDDING_Y});

    visualizer::write("island_astar.bmp", &m_oElevation[0], &m_oOverrides[0], asLocation, IMAGE_DIM);
}


TEST_F(cPlannerTest, island_map_astar_wiki)
{
    InitIslandMap();

    CreateRover(tLocation{ROVER_X, ROVER_Y}, tLocation{BACHELOR_X, BACHELOR_Y});

    m_poAudiRover->SetStart(tLocation{ROVER_X, ROVER_Y});
    m_poAudiRover->SetGoal(tLocation{BACHELOR_X, BACHELOR_Y});
    tResult sResultR2B = m_poAudiRover->Summon(1, 1, "ASTAR_WIKI");
    EXPECT_TRUE(sResultR2B.bFoundGoal);
    EXPECT_TRUE(sResultR2B.bConsistentHeuristic);


    m_poAudiRover->SetStart({BACHELOR_X, BACHELOR_Y});
    m_poAudiRover->SetGoal({WEDDING_X, WEDDING_Y});
    tResult sResultB2W = m_poAudiRover->Summon(1, 1, "ASTAR_WIKI");
    EXPECT_TRUE(sResultB2W.bFoundGoal);
    EXPECT_TRUE(sResultB2W.bConsistentHeuristic);


    //EXPECT_NEAR(fTimeAStar, fTimeAStarWiki, 1.0e-8);
    //EXPECT_NEAR(fTimeAStarWiki, fTimeAStarRBG, 1.0e-8);

    std::vector<tLocation> asLocation;
    asLocation.push_back(tLocation{ROVER_X, ROVER_Y});
    asLocation.push_back(tLocation{BACHELOR_X, BACHELOR_Y});
    asLocation.push_back(tLocation{WEDDING_X, WEDDING_Y});

    visualizer::write("island_astar_wiki.bmp", &m_oElevation[0], &m_oOverrides[0], asLocation, IMAGE_DIM);
}

TEST_F(cPlannerTest, island_map_astar_rbg)
{
    InitIslandMap();

    CreateRover(tLocation{ROVER_X, ROVER_Y}, tLocation{BACHELOR_X, BACHELOR_Y});

    m_poAudiRover->SetStart(tLocation{ROVER_X, ROVER_Y});
    m_poAudiRover->SetGoal(tLocation{BACHELOR_X, BACHELOR_Y});
    tResult sResultR2B = m_poAudiRover->Summon(1, 1, "ASTAR_RBG");
    EXPECT_TRUE(sResultR2B.bFoundGoal);
    EXPECT_TRUE(sResultR2B.bConsistentHeuristic);


    m_poAudiRover->SetStart({BACHELOR_X, BACHELOR_Y});
    m_poAudiRover->SetGoal({WEDDING_X, WEDDING_Y});
    tResult sResultB2W = m_poAudiRover->Summon(1, 1, "ASTAR_RBG");
    EXPECT_TRUE(sResultB2W.bFoundGoal);
    EXPECT_TRUE(sResultB2W.bConsistentHeuristic);


    //EXPECT_NEAR(fTimeAStar, fTimeAStarWiki, 1.0e-8);
    //EXPECT_NEAR(fTimeAStarWiki, fTimeAStarRBG, 1.0e-8);

    std::vector<tLocation> asLocation;
    asLocation.push_back(tLocation{ROVER_X, ROVER_Y});
    asLocation.push_back(tLocation{BACHELOR_X, BACHELOR_Y});
    asLocation.push_back(tLocation{WEDDING_X, WEDDING_Y});

    visualizer::write("island_astar_rbg.bmp", &m_oElevation[0], &m_oOverrides[0], asLocation, IMAGE_DIM);
}

TEST_F(cPlannerTest, island_map_all)
{
    InitIslandMap();

    CreateRover(tLocation{ROVER_X, ROVER_Y}, tLocation{BACHELOR_X, BACHELOR_Y});

    m_poAudiRover->SetStart({BACHELOR_X, BACHELOR_Y});
    m_poAudiRover->SetGoal({WEDDING_X, WEDDING_Y});

    tResult sResultAStar = m_poAudiRover->Summon(1, 1, "ASTAR");
    EXPECT_TRUE(sResultAStar.bFoundGoal);
    EXPECT_TRUE(sResultAStar.bConsistentHeuristic);
    double fTimeAStar = m_poAudiRover->TotalTime();
    m_poAudiRover->ResetTime();

    tResult sResultAStarWiki = m_poAudiRover->Summon(1, 1, "ASTAR_WIKI");
    EXPECT_TRUE(sResultAStarWiki.bFoundGoal);
    EXPECT_TRUE(sResultAStarWiki.bConsistentHeuristic);
    double fTimeAStarWiki = m_poAudiRover->TotalTime();
    m_poAudiRover->ResetTime();

    tResult sResultAStarRBG = m_poAudiRover->Summon(1, 1, "ASTAR_RBG");
    EXPECT_TRUE(sResultAStarRBG.bFoundGoal);
    EXPECT_TRUE(sResultAStarRBG.bConsistentHeuristic);
    double fTimeAStarRBG = m_poAudiRover->TotalTime();

    EXPECT_NEAR(fTimeAStar, fTimeAStarWiki, 1.0e-8);
    EXPECT_NEAR(fTimeAStarWiki, fTimeAStarRBG, 1.0e-8);

    std::vector<tLocation> asLocation;
    asLocation.push_back(tLocation{ROVER_X, ROVER_Y});
    asLocation.push_back(tLocation{BACHELOR_X, BACHELOR_Y});
    asLocation.push_back(tLocation{WEDDING_X, WEDDING_Y});

    visualizer::write("island_all.bmp", &m_oElevation[0], &m_oOverrides[0], asLocation, IMAGE_DIM);
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

TEST_F(cPlannerTest, heuristic)
{
    uint8_t nStepSize = 1;
    uint8_t nVelocity = 1; /// 1 cell per island second

    m_poAudiRover->InitializePlanner(nStepSize, nVelocity);
    cPlanner *poPlanner = static_cast<cPlanner*>(m_poAudiRover->GetPlanner());

    poPlanner->GenerateHeuristic();
}
*/