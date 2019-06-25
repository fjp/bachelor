//
// Created by Franz Pucher on 2019-05-18.
//

#ifndef BACHELOR_AUDI_ROVER_H
#define BACHELOR_AUDI_ROVER_H


#include "rover_interface.h"

#include "planner.h"

#include "graph.h"
#include "result.h"

namespace planner {


    ///\brief The concrete cAudiRover class implementation of the interface cRoverInterface<size_t Direction>.
    ///\details Provides eight actions and a summon feature.
    class cAudiRover : public cRoverInterface<8>, public std::enable_shared_from_this<cAudiRover> {

    public:
        ///\brief Receives the map data such as elevation and overrides and generates a map stored in m_oMap.
        ///\details The action struct array of the interface m_asActions is initialized here together with
        ///         the individual cost values.
        cAudiRover(uint8_t* i_oElevation,
                   uint8_t* i_oOverrides,
                   int i_nHeight, int i_nWidth);


        ///\brief The summon feature that the Audi rover provides
        ///\details Calls InitializePlanner which then acts as a factory to create a planner according to i_strAlgorithm parameter.
        ///         Sets the step and velocity to 1. Note: step size and velocity should always be set to 1.
        ///
        /// \param i_strAlgorithm Rvalue reference input specifies the algorithm to use. Possible strings "ASTAR", "ASTAR_WIKI", "ASTAR_RBG".
        /// \param i_nStepSize the step size of the rover. Should be set to 1.
        /// \param i_nVelocity the velocity of the rover. Deprecated. Should be set to 1.
        /// \return Planning result struct with information such as travelling time and heuristic consistency.
        tResult Summon(std::string&& i_strAlgorithm = "ASTAR", const int i_nStepSize = 1, const int i_nVelocity = 1);

        ///\brief Initializes the reference to the planner which is located in the inherited interface.
        ///\details Acts as a factory method to generate the planner which implements the passed algorithm i_strAlgorithm
        ///
        /// \param i_nStepSize the step size of the rover. Should be set to 1.
        /// \param i_nVelocity the velocity of the rover. Deprecated. Should be set to 1.
        /// \param i_strAlgorithm the algorithm to use, can be currently one of "ASTAR", "ASTAR_WIKI", "ASTAR_RBG".
        /// \return shared pointer to the created concrete planner class (cPlanner, cPlannerWiki, cPlannerRBG).
        std::shared_ptr<cPlannerInterface<8>> InitializePlanner(const int& i_nStepSize, const int& i_nVelocity, std::string&& i_strAlgorithm) override;


        ///\brief Destructor to delete the allocated memory of the map.
        ~cAudiRover() {
            //std::cout << "~cAudiRover" << std::endl;
        };

        ///\brief Total time in island seconds to travel on the fastest found path by AStar().
        double TotalTime() const;

        ///\brief Resets the total planning time to zero island seconds.
        void ResetTime();


        ///\brief Debug method to check use cound of shared pointer
        void countPlanner();


    private:
        ///\brief Reference to the map of type cGraph.
        std::shared_ptr<cGraph> m_poMap;

        ///\brief Used to calculate the total traveling time, depending on how often the Summon() method gets called.
        double m_fTotalTime;

    };

}


#endif //BACHELOR_AUDI_ROVER_H
