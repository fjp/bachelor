//
// Created by Franz Pucher on 2019-05-18.
//

#ifndef BACHELOR_PLANNERINTERFACE_H
#define BACHELOR_PLANNERINTERFACE_H

#include "graph.h"
#include "node.h"

#include <vector>
#include <memory>

namespace planner {

    /// Forward declaration of interface planner::cRoverInterface.
    template <size_t Directions>
    class cRoverInterface;


    ///\brief cPlannerInterface is an abstract interface which can be implemented by concrete planner classes.
    template <size_t Directions>
    class cPlannerInterface {

    public:

        ///\brief
        explicit cPlannerInterface(cRoverInterface<Directions> *i_poRover, cGraph &i_oMap) : m_poRover(i_poRover), m_oMap(i_oMap) {};

        virtual ~cPlannerInterface() {
            if (nullptr != m_poRover)
                delete m_poRover;
            m_poRover = nullptr;

        };

        virtual bool Plan() = 0;
        virtual void TraversePath(tNode *i_psNode) const = 0;


        virtual bool GoalTest(const tNode *i_sFirst, const tNode *i_sSecond) const = 0;
        virtual tNode* Child(tNode *i_sParent, const tAction &i_sAction) const = 0;


    protected:
        cRoverInterface<Directions> *m_poRover;
        cGraph &m_oMap;


    };


}

#endif //BACHELOR_PLANNERINTERFACE_H
