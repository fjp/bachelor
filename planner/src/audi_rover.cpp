//
// Created by Franz Pucher on 2019-05-18.
//

#include <cmath>
#include "audi_rover.h"

#include "planner_wiki.h"
#include "planner_rbg.h"

namespace planner
{

    cAudiRover::cAudiRover(
            uint8_t* i_oElevation,
            uint8_t* i_oOverrides,
            int i_nHeight, int i_nWidth) : cRoverInterface(), m_oElevation(i_oElevation), m_oOverrides(i_oOverrides)
            , m_poMap(nullptr), m_fTotalTime(0.f) {

        std::cout << "cAudiRover" << std::endl;

        m_poMap = std::make_shared<cGraph>(m_oElevation, m_oOverrides, i_nHeight, i_nWidth);


        /// Define direction and cost of different possible actions
        m_asActions[0] = tAction{ 1, 0, m_fCostStraight };   // E
        m_asActions[1] = tAction{ 1, -1, m_fCostDiagonal };  // NE
        m_asActions[2] = tAction{ 0, -1, m_fCostStraight };  // N
        m_asActions[3] = tAction{ -1, -1, m_fCostDiagonal }; // NW
        m_asActions[4] = tAction{ -1, 0, m_fCostStraight };  // W
        m_asActions[5] = tAction{ -1, 1, m_fCostDiagonal };  // SW
        m_asActions[6] = tAction{ 0, 1, m_fCostStraight };   // S
        m_asActions[7] = tAction{ 1, 1, m_fCostDiagonal };   // SE

    }

    tResult cAudiRover::Summon(const int i_nStepSize, const int i_nVelocity, std::string&& i_strAlgorithm) {

        auto poPlanner = InitializePlanner(i_nStepSize, i_nVelocity, std::move(i_strAlgorithm));

        if (poPlanner)
        {
            //poPlanner->SetAlgorithm(i_strAlgorithm);
            //countPlanner();
            m_fTotalTime += poPlanner->Plan();

            return poPlanner->Result();
        }
        return tResult{};

    }

    std::shared_ptr<cPlannerInterface<8>> cAudiRover::InitializePlanner(const int &i_nStepSize, const int &i_nVelocity, std::string&& i_strAlgorithm) {

        SetStepSize(i_nStepSize);

        SetVelocity(i_nVelocity);

        if ("ASTAR" == i_strAlgorithm) {
            auto poPlanner = std::make_shared<cPlanner>(shared_from_this(), m_poMap);
            m_poPlanner = poPlanner;
            return poPlanner;
        }
        if ("ASTAR_WIKI" == i_strAlgorithm) {
            auto poPlanner = std::make_shared<cPlannerWiki>(shared_from_this(), m_poMap);
            m_poPlanner = poPlanner;
            return poPlanner;
        }
        if ("ASTAR_RBG" == i_strAlgorithm) {
            auto poPlanner = std::make_shared<cPlannerRBG>(shared_from_this(), m_poMap);
            m_poPlanner = poPlanner;
            return poPlanner;
        }

        std::cout << "Failed to initialize planner with algorithm: " << i_strAlgorithm << std::endl;
        return nullptr;
    }

    void cAudiRover::countPlanner()
    {
        std::cout << "      m_poPlanner.use_count() == " << m_poPlanner.use_count() << ", expired: " << m_poPlanner.expired() << std::endl;
    }


    double cAudiRover::TotalTime() const {
        return m_fTotalTime;
    }

    void cAudiRover::ResetTime() {
        m_fTotalTime = 0.f;
    }

}


