//
// Created by Franz Pucher on 2019-05-18.
//

#ifndef BACHELOR_PLANNERINTERFACE_H
#define BACHELOR_PLANNERINTERFACE_H

#include "graph.h"
#include "node.h"

#include <vector>

namespace planner {

    /// Forward declaration of interface planner::cRoverInterface.
    template <size_t Directions>
    class cRoverInterface;


    ///\brief cPlannerInterface is an abstract interface which can be implemented by concrete planners classes.
    template <size_t Directions>
    class cPlannerInterface {

    public:

        ///\brief
        explicit cPlannerInterface(cRoverInterface<Directions> *i_oRover, cGraph &i_oMap) : m_oRover(i_oRover), m_oMap(i_oMap) {};

        virtual bool Plan() = 0;
        virtual void TraversePath(tNode *i_psNode) const = 0;


        virtual bool GoalTest(const tNode *i_sFirst, const tNode *i_sSecond) const = 0;
        virtual tNode* Child(tNode *i_sParent, const tAction &i_sAction) const = 0;


    protected:
        cRoverInterface<Directions> *m_oRover;
        cGraph &m_oMap;


    };


}

#endif //BACHELOR_PLANNERINTERFACE_H
