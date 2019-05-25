//
// Created by Franz Pucher on 2019-05-19.
//

#ifndef BACHELOR_GRAPH_H
#define BACHELOR_GRAPH_H

#include <stdint.h>

namespace planner {


    ///\brief The map class cMap has pointers to the elevation.data (m_poElevation) and overrides.data (m_poOverrides).
    ///\details This class is used to check for constraints in the map (e.g. water basins, rivers and marsh,
    ///         which cannot be traversed by the robot. The elevation data is used to model the acceleration of the rover.
    class cGraph {
    public:
        ///\brief Initializes the members m_poElevation, m_poOverrides and the maps height m_nHeight and width m_nWidth.
        cGraph(uint8_t *i_oElevation,
               uint8_t *i_oOverrides,
               int i_nHeight, int i_nWidth);


        ///\brief Get elevation at provided location
        ///\details From the read elevation.data the 8 bit value is returned.
        ///\param[in] i_nX X location of the elevation.
        ///\param[in] i_nY Y location of the elevation.
        ///\return Elevation at (i_nX, i_nY).
        uint8_t Elevation(int i_nX, int i_nY);

        ///\brief The overrides map describes the constraints, see \enum OverrideFlags in constants.h.
        ///\details From the read overrides.data the 8 bit value is returned.
        ///         \li 1. Rivers and marsh have bit 4 set.
        ///         \li 2. Water basins have bit 6 set.
        ///\param[in] i_nX X location of the elevation.
        ///\param[in] i_nY Y location of the elevation.
        ///\return Elevation at (i_nX, i_nY).
        uint8_t Overrides(int i_nX, int i_nY);

        ///\brief Check if the position is on mainland or water.
        ///\param[in] i_nX X coordinate to be checked for water.
        ///\param[in] i_nY Y coordinate to be checked for water.
        ///\return true if the provided position is on water, false otherwise.
        bool Water(int i_nX, int i_nY);

        ///\brief Check if the position between the two locations is on mainland or water or goes through it.
        ///\param[in] i_nX X coordinate to be checked for water.
        ///\param[in] i_nY Y coordinate to be checked for water.
        ///\return true if the provided position is on or goes through water, false otherwise.
        bool Water(int i_nX0, int i_nY0, int i_nX1, int i_nY1);

        ///\brief Getter for the height of the map, stored in m_nHeight.
        ///\return The total height of the map.
        int Height() const;

        ///\brief Getter for the width of the map, stored in m_nWidth.
        ///\return The total width of the map.
        int Width() const;


        ///\brief Sets the provided overrides m_poOverrides 8 bit value i_nValue at the provided location while keeping the old value (or operation).
        ///\param[in] i_nX X location of the overrides m_poOverrides that should be updated.
        ///\param[in] i_nY Y location of the overrides m_poOverrides that should be updated.
        ///\param[in] i_nValue The or operator is applied to this 8 bit mask value and the current overrides value at (i_nX, i_nY).
        void SetOverrides(int i_nX, int i_nY, uint8_t i_nValue);

    private:
    public: // TODO public for gtesting - use better design wiht Getter and Setter.
        ///\brief Elevation over the ground (0-255).
        uint8_t *m_oElevation;
        ///\brief Overrides is the terrain flag bit mask which act as constraints see \enum OverrideFlags and planner::cMap::Water().
        uint8_t *m_oOverrides;

        ///\brief Stores the height of the map.
        int m_nHeight;
        ///\brief Stores the width of the map.
        int m_nWidth;

    };
}


#endif //BACHELOR_GRAPH_H
