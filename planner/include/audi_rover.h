//
// Created by Franz Pucher on 2019-05-18.
//

#ifndef BACHELOR_AUDI_ROVER_H
#define BACHELOR_AUDI_ROVER_H


#include "rover_interface.h"

#include "planner.h"



namespace planner {


    class cAudiRover : public cRoverInterface {

    public:
        cAudiRover(std::vector<uint8_t> &i_oElevation,
                std::vector<uint8_t> &i_oOverrides);

        void SetStart(int i_nX, int i_nY);

        void SetGoal(int i_nX, int i_nY);

        void Summon();

        void SetCost(double i_nStraight, double i_nDiagonal);


    private:
        cPlanner *m_oPlanner;



    };

}


#endif //BACHELOR_AUDI_ROVER_H
