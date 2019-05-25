//
// Created by Franz Pucher on 2019-05-18.
//

#ifndef BACHELOR_PLANNER_H
#define BACHELOR_PLANNER_H


#include "planner_interface.h"

#include "priority_queue.h"

#include "graph.h"

#include <vector>


///\brief Contains the Interfaces and their implementations to solve the Bachelor challenge.
namespace planner {


    ///\brief Implements the planner interface cPlannerInterface<size_t Directions> with an action vector of size eight.
    class cPlanner : public cPlannerInterface<8> {
    public:

        ///\brief Initializes member variables m_poRover and m_oMap and calls CalculateConsistencyFactor().
        ///\details The
        cPlanner(cRoverInterface<8> *i_poRover, cGraph &i_oMap);

        ///\brief Destructor to delete the allocated memory.
        ~cPlanner() {

        };

        ///\brief Override of the base interface cPlannerInterface, which invokes the AStar() search algorithm.
        ///\returns the time to travel from start to goal if it was found. Otherwise -1 is returned.
        float Plan() override;


        ///\brief Types of heuristics that can be calculated with UpdateHeuristic()
        enum tHeuristic { MANHATTEN, EUCLIDEAN, OCTILE, CHEBYSHEV };

        /// \brief Output distance heuristic map to file, which is used to generate the Matlab plot.
        /// \details This implementation calls UpdateHeuristic() to calculate a distance heuristic. Because the robot can move in
        ///          eight directions the octile distance heuristic is calculated.
        ///          Other possible gird map heuristics are Manhatten, Chebyshev and Euclidean.
        void GenerateHeuristic();

        ///\brief Updates the heuristic value of the node argument i_sNode.
        ///\details Calcualtes a grid map distance heuristic. Can be one of the heuristics defined in tHeuristic.
        ///\param[in] i_sNode the node which heuristic is updated
        ///\param[in] i_eHeuristic the type of heuristic to calculate, see tHeuristic.
        ///\return The calculated heuristic value.
        float UpdateHeuristic(tNode *i_sNode, const tHeuristic i_eHeuristic = OCTILE) const;


        ///\brief Updates the heuristic value of the node located at tlocation i_sLocation.
        ///\details Calcualtes a grid map distance heuristic. Can be one of the heuristics defined in tHeuristic.
        ///\param[in] i_sNode the node which heuristic is updated
        ///\param[in] i_eHeuristic the type of heuristic to calculate, see tHeuristic.
        ///\return The calculated heuristic value.
        float UpdateHeuristic(const tLocation &i_sLocation, const tHeuristic i_eHeuristic = OCTILE) const;



        ///\brief Check if the heuristic of node i_sNode is consistent
        ///\details Consistency is given if h(n) <= c(n,p) + h(p), where h(p) is the heuristic of the parent node
        ///         and c(n,p) are the step costs from parent p to node n.
        ///\param[in] i_sNode the node which heuristic value is tested.
        void HeuristicCheck(tNode *i_sNode) const;

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
        ///\param[in] i_sLocation the location of type tLocation.
        ///\returns true if the location i_sLocation lies within the map otherwise false.
        bool WithinMap(const tLocation &i_sLocation) const;

        ///\brief Goal test to check if the two provided nodes i_sFirst, i_sSecond are equal.
        ///\details Note that this check takes the step size m_nStepSize of the rover into account.
        ///         This allows to set a step size greater than one, which can be used for debugging.
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
        const int GradX(int i_nX, int i_nY) const;

        ///\brief Calculates the discrete gradient of the map m_poMap in y direction.
        const int GradY(int i_nX, int i_nY) const;

        ///\brief Calculates the node hash using its location and the width of the map
        ///\details The hash is required to sort the std::map<tNode> oCost of reaching a node,
        ///         which is used in the AStar() search algorithm.
        uint32_t NodeHash(const tNode *i_sNode) const;

    private:

        ///\brief AStar algorithm implementation.
        ///\details Initializes start, goal and intermediate nodes (sCurrent and sNext). The frontier m_oFrontier
        ///         is implemented as a priority queue PriorityQueue<tNode*> and initialized with the start node.
        ///         All other expanded nodes are store in a std::map oPathCost with their currently best g score value.
        ///         The algorithm makes use of GoalTest(), Child() to generate sucessor nodes given an action,
        ///         Traversable to check for the constraints imposed by the overrides.data file, cost methods
        ///         UpdateCost() for the step cost, UpdateHeuristic() while checking for consistency HeuristicCheck().
        ///         In case the goal node is reached, the method TraversePath() is invoked to move from the goal back
        ///         to the start node, thereby following the fastest path and setting bit 1 of the overrides map, see
        ///         planner::cGraph::SetOverrides().
        ///\returns the time it took to find the fastest path in island seconds.
        float AStar();

        ///\brief Calculates a consistency factor to get a consistent heuristic h(n) <= c(p,n) + h(p)
        ///\details Calculates the gradient of the elevation and considers the acceleration on slopes.
        ///         The result is stored in members m_nMaxGradient and m_fConsistencyFactor.
        void CalculateConsistencyFactor();

        ///\brief Maximum gradient of the elevation, used to normalize the heuristic values. See UpdateHeuristic(tNode *i_sNode).
        int m_nMaxGradient;

        ///\brief This value is calculated in the constructor of planner::cPlanner and used to scale the heuristic values to get consistency.
        float m_fConsistencyFactor;


        ///\brief Priority queue data structure, which is the basis of A star. Always deques the node with the best f score first.
        PriorityQueue<tNode*, double> m_oFrontier;


        ///\brief Debug method to plot intermediate paths during planning. Used in Plan().
        void Plot();



    };





}

#endif //BACHELOR_PLANNER_H
