//
// Created by Franz Pucher on 2019-05-21.
//

#ifndef BACHELOR_PRIORITY_QUEUE_H
#define BACHELOR_PRIORITY_QUEUE_H


#include <queue>
#include <tuple>
#include <algorithm>

namespace planner {


    ///\brief Priority queue servers as frontier that stores the best nodes explored during planner::cPlanner::AStar().
    ///\details The priority queue is sorted by the evaluation function value \f$f(n)\f$.
    ///         Code from https://www.redblobgames.com/pathfinding/a-star/implementation.html#cplusplus
    ///
    /// \tparam T element of the queue such as a node with location information.
    /// \tparam priority_t specifies the importance of a node (f score value).
    template<typename T, typename priority_t>
    struct PriorityQueue {
        typedef std::pair<priority_t, T> PQElement;
        std::priority_queue<PQElement, std::vector<PQElement>,
                std::greater<PQElement>> oElements;

        ///\brief Check if the queue is empty, which is used get new best nodes or resign because no path to the goal was found.
        inline bool empty() const {
            return oElements.empty();
        }

        ///\brief Adds new nodes to the priority queue, which is used when their path costs are lower than previously stored, see planner::cPlanner::AStar().
        inline void put(T item, priority_t priority) {
            oElements.emplace(priority, item);
        }

        ///\brief Get the best element in the queue and remove it.
        T get() {
            T oBestElement = oElements.top().second;
            oElements.pop();
            return oBestElement;
        }

        ///\brief Get the best element from the queue without removing it.
        T pop() {
            T oBestElement = oElements.top().second;
            return oBestElement;
        }

        ///\brief Clear all elements by overriding the struct field oElements.
        void clear() {
            oElements = std::priority_queue<PQElement, std::vector<PQElement>,
                    std::greater<PQElement>>();
        }
    };
}

#endif //BACHELOR_PRIORITY_QUEUE_H
