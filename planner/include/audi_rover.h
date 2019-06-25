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
        tResult Summon(const int i_nStepSize = 1, const int i_nVelocity = 1, std::string&& i_strAlgorithm = "ASTAR");

        ///\brief Initializes the reference to the planner which is located in the inherited interface.
        ///\details
        std::shared_ptr<cPlannerInterface<8>> InitializePlanner(const int &i_nStepSize, const int &i_nVelocity, std::string&& i_strAlgorithm) override;


        ///\brief Destructor to delete the allocated memory of the map.
        ~cAudiRover() {
            //std::cout << "~cAudiRover" << std::endl;
        };

        void countPlanner();


    private:
        ///\brief Reference to the map of type cGraph.
        std::shared_ptr<cGraph> m_poMap;

        ///\brief Pointer to the elevation data (rather redundant because it is stored in the map)
        uint8_t* m_oElevation;
        ///\brief Pointer to the overrides data, which will also be stored in the map m_oMap.
        uint8_t* m_oOverrides;

        ///\brief Used to calculate the total traveling time, depending on how often the Summon() method gets called.
        double m_fTotalTime;
    public:
        ///\brief Total time in island seconds to travel on the fastest found path by AStar().
        double TotalTime() const;

        ///\brief Resets the total planning time to zero island seconds.
        void ResetTime();

    };

}


#endif //BACHELOR_AUDI_ROVER_H
