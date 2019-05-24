//
// Created by Franz Pucher on 2019-05-18.
//

#ifndef BACHELOR_PLANNER_H
#define BACHELOR_PLANNER_H


#include "planner_interface.h"

#include "priority_queue.h"

#include "map.h"

#include <vector>
#include <iostream> // TODO remove


namespace planner {


    class cPlanner : public cPlannerInterface<8> {
    public:
        cPlanner(cRoverInterface<8> *i_poRover, cMap &i_oMap);

        ///\brief
        bool Plan() override;


        ~cPlanner();


        enum tHeuristic { MANHATTEN, EUCLIDEAN, OCTILE, CHEBYSHEV };

        /// \brief Generate distance heuristic vector member, which is inherited from the cPlannerInterface.
        /// \details This implementation calculates the octile distance heuristic because the robot can move in
        ///          eight directions. Other possible gird map heuristics are Manhatten, Chebyshev and Euclidean.
        void GenerateHeuristic();

        ///\brief Updates the heuristic value of the node argument i_sNode.
        ///\details Calcualtes a grid map distance heuristic. Can be one of the heuristics defined in tHeuristic.
        ///\param[in] i_sNode the node which heuristic is updated
        ///\param[in] i_eHeuristic the type of heuristic to calculate, see tHeuristic.
        float UpdateHeuristic(tNode *i_sNode, const tHeuristic i_eHeuristic = OCTILE) const;

        float UpdateHeuristic(const tLocation &i_sLocation, const tHeuristic i_eHeuristic = OCTILE) const;



        void HeuristicCheck(tNode *i_sNode) const {
            float fStepCost = i_sNode->g - i_sNode->psParent->g;
            //if (!(Heuristic(sNext) <= StepCost + Heuristic(sNext->psParent)))
            if (!(i_sNode->h <= fStepCost + i_sNode->psParent->h))
            {
                std::cout << "Heuristic not consistent: " << i_sNode->h << " > " << fStepCost << " + " << i_sNode->psParent->h << std::endl;
            }
        };

        ///\brief Updates the node argument with its path cost \f$g(n)\f$ with island seconds as its unit.
        ///\details Uses the slope found from the gradient of the elevation map cMap::m_oElevation to calculate
        ///         an acceleration value, where only its component in the x,y plane is used.
        ///         The value is added to the time it takes for a straight or diagonal step (depending on the action of the node).
        ///         The sum is stored in \f$g(n)\f$ of the node planner::tNode i_sNode.
        ///\param[in] i_sNode The node which path cost is updated.
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
        ///         which is used in the AStar() search algorithm.
        uint32_t NodeHash(const tNode *i_sNode) const;

    private:

        bool AStar();

        ///\brief Maximum gradient of the elevation, used to normalize the heuristic values. See UpdateHeuristic(tNode *i_sNode).
        uint8_t m_nMaxGradient;

        ///\brief This value is calculated in the constructor of planner::cPlanner and used to scale the heuristic values.
        float m_fConsistencyFactor;

        ///\brief Two by two matrix containing the heuristic values, which are generated in GenerateHeuristic().
        //std::vector<std::vector<float> > m_mfHeuristic;

        ///\brief Priority queue data structure, which is the basis of A star. Always deques the node with the best f score first.
        PriorityQueue<tNode*, double> m_oFrontier;


        ///\brief Debug method to plot intermediate paths during planning. Used in Plan()
        void Plot();



    };





}

#endif //BACHELOR_PLANNER_H
