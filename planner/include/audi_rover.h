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


        //void SetStart(int i_nX, int i_nY);

        //void SetGoal(int i_nX, int i_nY);



        ///\brief The summon feature that the Audi rover provides
        void Summon(uint8_t i_nStepSize = 1, uint8_t i_nVelocity = 1);



    private:
        //cPlannerInterface<8> *m_oPlanner;

        cGraph *m_oMap; // TODO change to cMap

        uint8_t* m_oElevation;
        uint8_t* m_oOverrides;



    };

}


#endif //BACHELOR_AUDI_ROVER_H
