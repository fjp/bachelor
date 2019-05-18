//
// Created by Franz Pucher on 2019-05-18.
//

#ifndef BACHELOR_AUDI_ROVER_H
#define BACHELOR_AUDI_ROVER_H


#include "rover_interface.h"

#include "planner.h"

#include "priority_queue.h"

#include "structs.h"

#include <vector>

namespace planner {


    //template <typename T>
    class cAudiRover : public cRoverInterface<std::vector<uint8_t>, cPlanner, tLocation > {

    public:
        cAudiRover(std::vector<uint8_t> &i_oElevation, std::vector<uint8_t> &i_oOverrides,
                cPlanner& i_oPlanner);

        void SetLocation(tLocation i_x, tLocation i_y) override;

    };

}


#endif //BACHELOR_AUDI_ROVER_H
