//
// Created by Franz Pucher on 2019-05-18.
//

#ifndef BACHELOR_PLANNER_H
#define BACHELOR_PLANNER_H


#include "planner_interface.h"

#include "priority_queue.h"

#include "graph.h"

#include <vector>
#include <iostream> // TODO remove


namespace planner {


    class cPlanner : public cPlannerInterface<8> {
    public:
        cPlanner(cRoverInterface<8> *i_oRover, cGraph &i_oMap);

        bool Plan() override;




        /// \brief Generate distance heuristic vector member, which is inherited from the cPlannerInterface.
        /// \details This implementation calculates the octile distance heuristic because the robot can move in
        ///          eight directions. Other possible gird map heuristics are Manhatten, Chebyshev and Euclidean.
        void GenerateHeuristic();


        ///\brief
        void UpdateHeuristic(tNode *i_sNode) const {
            i_sNode->h = m_mnHeuristic[i_sNode->sLocation.nX][i_sNode->sLocation.nY] * 1.f/m_nMaxGradient;
        };

        void HeuristicCheck(tNode *i_sNode) const {
            float fStepCost = i_sNode->g - i_sNode->psParent->g;
            //if (!(Heuristic(sNext) <= StepCost + Heuristic(sNext->psParent)))
            if (!(i_sNode->h <= fStepCost + i_sNode->psParent->h))
            {
                std::cout << "Heuristic not consistent: " << i_sNode->h << " > " << fStepCost << " + " << i_sNode->psParent->h << std::endl;
            }
        };

        int Heuristic(tNode *i_sNode) const {
            return m_mnHeuristic[i_sNode->sLocation.nX][i_sNode->sLocation.nY];
        }

        void UpdateCost(tNode *i_sNode) const;


        ///\brief Test if the provided location i_sLocation lies within the map
        ///\details Checks if the provided location lies within the map height and width
        ///         and if the location is equal or greater than zero.
        bool WithinMap(const tLocation &i_sLocation) const;

        ///\brief Goal test to check if the two provided nodes i_sFirst, i_sSecond are equal.
        ///\details Note that this check takes the step size m_nStepSize of the rover into account.
        ///\param[in] i_sFirst could be the current node that needs to be checked.
        ///\param[in] i_sSecond could be the goal node.
        ///\return true or false if the two nodes are equal.
        virtual bool GoalTest(const tNode *i_sFirst, const tNode *i_sSecond) const override;

        ///\brief Generate a successor node state given a node i_sParent and action i_sAction.
        ///\details Overrides method of the base class inteface cPlannerInterface<size_t Directions>.
        ///         Defines a new node on the heap and initializes it according to the given action.
        ///
        ///\param[in] i_sParent node which becomes the parent of the new node.
        ///\param[in] i_sAction struct of type tAction that contains the direction and cost of the action.
        tNode* Child(tNode *i_sParent, const tAction &i_sAction) const override;

        ///\brief considers step size of rover and checks if the path is traversable TODO improve comment
        bool Traversable(tNode *i_sCurrent, tNode *i_sNext) const;

        ///\brief Given the node i_psNode the overrides map m_poOverrides is updated for displaying the path.
        ///\details Traversing a path takes place using the m_psParent field of the tNode struct.
        ///\param[in] i_psNode Goal node or any other which is traversed back
        void TraversePath(tNode *i_psNode) const override;

        ///\brief Calculates the discrete gradient of the map m_poMap in x direction.
        const int32_t GradX(uint32_t i_nX, uint32_t i_nY) const;

        ///\brief Calculates the discrete gradient of the map m_poMap in y direction.
        const int32_t GradY(uint32_t i_nX, uint32_t i_nY) const;

        ///\brief Calculates the node hash using its location and the width of the map
        ///\details The hash is required to sort the std::map<tNode> oCost of reaching a node,
        ///         which is used in the Astar() search algorithm.
        uint32_t NodeHash(const tNode *i_sNode) const;

    private:

        uint8_t m_nMaxGradient; ///< Maximum gradient of the map elevation.

        std::vector<std::vector<int> > m_mnHeuristic;

        PriorityQueue<tNode*, double> m_oFrontier;


        void Plot();



    };





}

#endif //BACHELOR_PLANNER_H
