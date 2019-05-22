//
// Created by Franz Pucher on 2019-05-18.
//

#ifndef BACHELOR_PLANNER_H
#define BACHELOR_PLANNER_H


#include "planner_interface.h"

#include "graph.h"

#include <vector>
#include <iostream> // TODO remove


namespace planner {


    class cPlanner : public cPlannerInterface<8> {
    public:
        cPlanner(cRoverInterface<8> *i_oRover, cGraph &i_oMap, uint8_t i_oStepSize = 1);

        void Plan() override;
        void PlanClean();



        /// \brief Generate distance heuristic vector member, which is inherited from the cPlannerInterface.
        /// \details This implementation calculates the octile distance heuristic because the robot can move in
        ///          eight directions. Other possible gird map heuristics are Manhatten, Chebyshev and Euclidean.
        void GenerateHeuristic();

    private:
        uint8_t m_nStepSize;
        uint8_t m_nMaxGradient; ///< Maximum gradient of the map elevation.

        std::vector<std::vector<int> > m_mnHeuristic;
    public:
        const int32_t &Heuristic(const uint i_nX, const uint i_nY) const;
        const int32_t &Heuristic(const tLocation &i_sLocation) const;

        //bool GoalTest(const tNode &i_sFirst, const tNode &i_sSecond) const override;
        //tNode Child(tNode &i_sParent, const tAction &i_sAction) override;

        ///\brief Test if the provided location i_sLocation lies within the map
        ///\details Checks if the provided location lies within the map height and width
        ///         and if the location is equal or greater than zero.
        bool WithinMap(const tLocation &i_sLocation) const;

        ///\brief Goal test to check if the two provided nodes i_sFirst, i_sSecond are equal.
        ///\details Note that this check takes the step size m_nStepSize of the rover into account.
        ///\param[in] i_sFirst could be the current node that needs to be checked.
        ///\param[in] i_sSecond could be the goal node.
        ///\return true or false if the two nodes are equal.
        bool GoalTest(const tNode *i_sFirst, const tNode *i_sSecond) const override;

        ///\brief Generate a successor node state given a node i_sParent and action i_sAction.
        ///\details Overrides method of the base class inteface cPlannerInterface<size_t Directions>.
        ///         Defines a new node on the heap and initializes it according to the given action.
        ///
        ///\param[in] i_sParent node which becomes the parent of the new node.
        ///\param[in] i_sAction struct of type tAction that contains the direction and cost of the action.
        tNode* Child(tNode *i_sParent, const tAction &i_sAction) const override;

    private:

        ///\brief Calculates the discrete gradient of the map m_poMap in x direction.
        const int32_t GradX(uint32_t i_nX, uint32_t i_nY) const;
        
        ///\brief Calculates the discrete gradient of the map m_poMap in y direction.
        const int32_t GradY(uint32_t i_nX, uint32_t i_nY) const;
        
        ///\brief Calculates the node hash using its location and the width of the map
        ///\details The hash is required to sort the std::map<tNode> oCost of reaching a node,
        ///         which is used in the Astar() search algorithm.
        uint32_t NodeHash(const tNode *i_sNode);

    };





}

#endif //BACHELOR_PLANNER_H
