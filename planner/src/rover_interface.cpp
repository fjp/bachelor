//
// Created by Franz Pucher on 2019-05-18.
//

#include "rover_interface.h"

namespace planner
{


    cRoverInterface::cRoverInterface()
    : m_astrMovementArrows{ "^", "<", "v", ">" },
      m_mnMovements{
              { 1, 0 },   // E
              { 1, 1 },   // NE
              { 0, 1 },   // N
              { -1, 1 },  // NW
              { -1, 0 },  // W
              { -1, -1 }, // SW
              { 0, -1 },  // S
              { 1, -1 },  // SE
      } {

    }
}