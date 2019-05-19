//
// Created by Franz Pucher on 2019-05-18.
//

#ifndef BACHELOR_AUDI_ROVER_H
#define BACHELOR_AUDI_ROVER_H


#include "rover_interface.h"

#include "planner.h"

#include "graph.h"

namespace planner {


    class cAudiRover : public cRoverInterface<8> {

    public:
        cAudiRover(uint8_t* i_oElevation,
                   uint8_t* i_oOverrides,
                   int i_nHeight, int i_nWidth);


        void SetStart(int i_nX, int i_nY);

        void SetGoal(int i_nX, int i_nY);



        void Summon();

        void SetCost(double i_nStraight, double i_nDiagonal);


    private:
        cPlannerInterface<8> *m_oPlanner;

        cGraph *m_oMap;

        uint8_t* m_oElevation;
        uint8_t* m_oOverrides;



    };

}


#endif //BACHELOR_AUDI_ROVER_H
