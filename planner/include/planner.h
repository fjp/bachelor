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
        cPlanner(std::shared_ptr<cRoverInterface<8>> i_poRover, std::shared_ptr<cGraph> i_oMap);

        ///\brief Destructor to delete the allocated memory.
        ~cPlanner() {
            //std::cout << "~cPlanner" << std::endl;
        };

        ///\brief Override of the base interface cPlannerInterface, which invokes the AStar() search algorithm.
        ///\returns the time to travel from start to goal if it was found. Otherwise -1 is returned.
        tResult Plan() override;


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
        double UpdateHeuristic(std::shared_ptr<tNode> i_psNode, const tHeuristic i_eHeuristic = OCTILE) const;


        ///\brief Updates the heuristic value of the node located at tlocation i_sLocation.
        ///\details Calcualtes a grid map distance heuristic. Can be one of the heuristics defined in tHeuristic.
        ///\param[in] i_sNode the node which heuristic is updated
        ///\param[in] i_eHeuristic the type of heuristic to calculate, see tHeuristic.
        ///\return The calculated heuristic value.
        double Heuristic(const tLocation &i_sLocation, const tHeuristic i_eHeuristic = OCTILE) const;


        ///\brief Check if the heuristic of node i_sNode is consistent
        ///\details Consistency is given if h(x) <= d(x,y) + h(y), where h(x) is the heuristic of the parent node
        ///         and d(x,y) are the step costs from parent x to node y.
        ///         In case the heuristic is not consistent the member m_sResult.bConsistentHeuristic is set to false.
        ///\param[in] i_sNode the node which heuristic value is tested.
        void HeuristicCheck(std::shared_ptr<tNode>& i_sNode);

        ///\brief Updates the node argument with its path cost \f$g(n)\f$ with island seconds as its unit.
        ///\details Uses the slope found from the gradient of the elevation map cMap::m_oElevation to calculate
        ///         an acceleration value, where only its component in the x,y plane is used.
        ///         The value is added to the time it takes for a straight or diagonal step (depending on the action of the node).
        ///         The sum is stored in \f$g(n)\f$ of the node planner::tNode i_sNode.
        ///\param[in] i_sNode The node which path cost is updated.
        void UpdateCost(std::shared_ptr<tNode> io_psNode) const;

        ///\brief Calculates the height cost which is added or subtracted from the step cost.
        ///\details In case the rover is moving uphill, this method calculates a height cost which is a percentage value
        ///         of the step cost of the current action. When the rover moves downhill, it is faster and therefore
        ///         the height cost is a negative value which is also a result of a percentage value of the step cost.
        ///         The percentage model uses the height difference while moving from i_sCurrent to i_sNext location.
        ///         This delta height, eigher positive or negative, is then normalized by the maximum height difference
        ///         the rover is able to move (climb or fall), which is defined to be 255 (max evlevation of a map).
        ///         \code{.cpp}
        ///         fHeightCost = i_sAction.fCost * fDeltaHeight / 255.f;
        ///         \endcode
        ///
        /// \tparam TLocation Location type that should contain two integer members nX and nY that describe the location.
        /// \param[in] i_sCurrent Location of the current node.
        /// \param[in] i_sNext Location of the node where the rover is moving to.
        /// \param[in] i_sAction the action the rover takes to get from i_sCurrent to i_sNext. Contains the step cost, which is used to calculate a percentage height cost.
        /// \return The height cost of moving from i_sCurrent to i_sNext. Will be less than or equal to the action cost, which is required to ensure a consistent heuristic.
        template<typename TLocation>
        double HeightCost(TLocation& i_sCurrent, TLocation& i_sNext, tAction& i_sAction) const
        {
            /// If the rover is going up or down hill, calculate the acceleration on the inclined plane
            /// Calculate current gradient in step direction
            double fDeltaHeight =
                    (m_oMap->Elevation(i_sNext.nX, i_sNext.nY) -
                     m_oMap->Elevation(i_sCurrent.nX, i_sCurrent.nY));

            /// This cost value is either positive or negative, depending on the height difference
            double fHeightCost = i_sAction.fCost * fDeltaHeight / 255.f; //static_cast<double>(m_nMaxGradient + 1); //10.f;

            return fHeightCost;
        };


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
        virtual bool GoalTest(std::shared_ptr<tNode>& i_sFirst, std::shared_ptr<tNode>& i_sSecond) const override;

        ///\brief Generate a successor node state given a node i_sParent and action i_sAction.
        ///\details Overrides method of the base class inteface cPlannerInterface<size_t Directions>.
        ///         Defines a new node on the heap and initializes it according to the given action.
        ///
        ///\param[in] i_sParent node which becomes the parent of the new node.
        ///\param[in] i_sAction struct of type tAction that contains the direction and cost of the action.
        ///\return the generated child node as shared pointer.
        std::shared_ptr<tNode> Child(std::shared_ptr<tNode>& i_sParent, const tAction &i_sAction) const override;

        ///\brief Checks if the path from i_sCurrent to i_sNext is traversable.
        ///\details Takes into account the set step size of the rover.
        ///
        /// \param i_sCurrent source node is the start of the path, going to i_sNext, that is going to be check.
        /// \param i_sNext destination node is the goal of the path, starting at i_sCurrent, that is going to be checked.
        /// \return true if the rover can move on the path between i_sCurrent and i_sNext. false otherwise.
        bool Traversable(std::shared_ptr<tNode> i_sCurrent, std::shared_ptr<tNode> i_sNext) const;

        ///\brief Reconstructs the best path, considers step size of rover.
        ///\details The nodes in the best path contains a field tNode::psParent which makes it possible to move back to
        ///          the start node, which has its parent pointer set to nullptr.
        ///          Given the node i_psNode the overrides map m_poOverrides is updated for displaying the path.
        ///          This method is also used in planner::cPlanner::Plot() to output intermediate paths on an output image.
        ///\param[in] The node at which we start to walk back. Usually the goal node.
        void TraversePath(std::shared_ptr<tNode>& i_psNode);

        ///\brief Calculates the discrete gradient of the map m_poMap in x direction.
        const int GradX(int i_nX, int i_nY) const;

        ///\brief Calculates the discrete gradient of the map m_poMap in y direction.
        const int GradY(int i_nX, int i_nY) const;

        ///\brief Calculates the node hash using its location and the width of the map
        ///\details The hash is required to sort the std::map<tNode> oCost of reaching a node,
        ///         which is used in the AStar() search algorithm.
        int NodeHash(std::shared_ptr<tNode>& i_sNode) const;


    protected:

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
        tResult AStar();


        ///\brief Calculates a consistency factor to get a consistent heuristic h(n) <= c(p,n) + h(p)
        ///\details Calculates the gradient of the elevation and considers the acceleration on slopes.
        ///         The result is stored in members m_nMaxGradient and m_fConsistencyFactor.
        void CalculateConsistencyFactor();

        ///\brief Maximum gradient of the elevation, used to normalize the heuristic values. See UpdateHeuristic(tNode *i_sNode).
        int m_nMaxGradient;

        ///\brief This value is calculated in the constructor of planner::cPlanner and used to scale the heuristic values to get consistency.
        double m_fConsistencyFactor;

        ///\brief Priority queue data structure, which is the basis of A star. Always deques the node with the best f score first.
        PriorityQueue<std::shared_ptr<tNode>, double> m_oFrontier;

        ///\brief Debug method to plot intermediate paths during planning. Used in Plan().
        void Plot();

        ///\brief Report travelling time in island seconds, minutes and hours.
        void PrintTravelResult();
    };
}

#endif //BACHELOR_PLANNER_H
