//
// Created by Franz Pucher on 2019-05-18.
//

#ifndef BACHELOR_PLANNER_WIKI_H
#define BACHELOR_PLANNER_WIKI_H


#include "planner.h"

///\brief Contains the Interfaces and their implementations to solve the Bachelor challenge.
namespace planner {


    ///\brief Extends the planner class to implement A* from Wikipedia
    class cPlannerWiki : public cPlanner {
    public:

        ///\brief Initializes member variables m_poRover and m_oMap and calls CalculateConsistencyFactor().
        ///\details The
        cPlannerWiki(std::shared_ptr<cRoverInterface<8>> i_poRover, std::shared_ptr<cGraph> i_oMap);

        ///\brief Destructor to delete the allocated memory.
        ~cPlannerWiki() {
            //std::cout << "~cPlannerWiki" << std::endl;
        };

        ///\brief Override of the base interface cPlannerInterface, which invokes the AStar() search algorithm.
        ///\returns the time to travel from start to goal if it was found. Otherwise -1 is returned.
        tResult Plan() override;


        ///\brief Given the node i_psNode the overrides map m_poOverrides is updated for displaying the path.
        ///\details Traversing a path takes place using the m_psParent field of the tNode struct.
        ///\param[in] i_psNode Goal node or any other which is traversed back
        template<typename TCostSoFar, typename TCameFrom>
        void ReconstructPath(TCostSoFar&& i_cost_so_far, TCameFrom&& i_came_from);


    protected:
        tResult AStar();
    };
}

#endif //BACHELOR_PLANNER_WIKI_H
