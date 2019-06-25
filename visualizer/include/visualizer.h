#ifndef __VISUALIZER_H__
#define __VISUALIZER_H__

#include <vector>
#include <functional>
#include <ostream>

#include <fstream>


#include "constants.h"

///\brief Contains functions for plotting overrides data and elevation.
namespace visualizer {

    /// Pixel values
    enum ImagePixelValues
    {
        IPV_PATH = 0,               // Results in red in the BMP
        IPV_WATER = 1,              // Results in the water color in the BMP
        IPV_VISITED = 2,
        IPV_ELEVATION_BEGIN = 3     // 2-255
    };

    ///\brief Used for the flags to specify what to visualise in visualizer::write
    enum tVisualise
    {
        LOCATIONS = 0x01,
        PATH = 0x02,
        LOCATIONS_PATH = 0x03,
        VISITED = 0x04,
        ALL = 0x07
    };

    /**
     * A method to write BMP file contents to a specified ostream.
     *
     * @param out The ostream to use for output. Could be directed into anything
     * @param elevationData Pointer to grid of elevation values. There must be width * height such
     *        elevation points.
     * @param width The width of the image
     * @param height The height of the image
     * @param pixelFilter A passed function or lambda that can change the pixel colormap index at passed
     *        x (from the left), and y (from the top) position of the elevationData. See enum
     *        ImagePixelValues above for interesting values to return.
     */
    void writeBMP(
        std::ostream& out,
        const uint8_t* elevationData,
        size_t width,
        size_t height,
        std::function<uint8_t(size_t, size_t, uint8_t)> pixelFilter);


    ///\brief Used in the functional from writeBMP to mark a node given its location.
    bool donut(int i_nX, int i_nY, int i_nX1, int i_nY1);

    ///\brief Used in the functional from writeBMP to plot the fastest path found by AStar() in cPlanner.
    bool path(int i_nX, int i_nY, uint8_t* overrides, int i_nImageDim = IMAGE_DIM, int i_nPenSize = 1);

    ///\brief Used in the functional from writeBMP to plot the visited nodes found by AStar() in cPlanner.
    bool visited(int i_nX, int i_nY, uint8_t* overrides, int i_nImageDim = IMAGE_DIM);


    template<typename TLocations>
    void write(std::string i_strName
            , uint8_t* i_oElevation
            , uint8_t* i_oOverrides
            , std::vector<TLocations> i_asLocation
            , int i_nImageDim = IMAGE_DIM
            , int i_nVisualise = ALL
            , int i_nPenSize = 1)
    {
        std::ofstream of(i_strName, std::ofstream::binary);
        visualizer::writeBMP(
                of,
                i_oElevation,
                i_nImageDim,
                i_nImageDim,
                [&] (size_t i_nX, size_t i_nY, uint8_t elevation) {

                    /// Marks interesting positions on the map (enable using flag)
                    if (i_nVisualise & LOCATIONS) {
                        for (auto sLocation : i_asLocation)
                        {
                            if (visualizer::donut(i_nX, i_nY, sLocation.nX, sLocation.nY))
                            {
                                return uint8_t(visualizer::IPV_PATH);
                            }
                        }
                    }

                    /// Visualize the found path (enable using flag)
                    if (i_nVisualise & PATH)
                    {
                        if (visualizer::path(i_nX, i_nY, i_oOverrides, i_nImageDim, i_nPenSize))
                        {
                            return uint8_t(visualizer::IPV_PATH);
                        }
                    }

                    /// Signifies water
                    if ((i_oOverrides[i_nY * i_nImageDim + i_nX] & (OF_WATER_BASIN | OF_RIVER_MARSH)) ||
                        elevation == 0)
                    {
                        return uint8_t(visualizer::IPV_WATER);
                    }

                    /// Signifies visited locations (enable using flag)
                    if (i_nVisualise & VISITED)
                    {
                        if (visualizer::visited(i_nX, i_nY, i_oOverrides, i_nImageDim))
                        {
                            return uint8_t(visualizer::IPV_VISITED);
                        }
                    }

                    /// Signifies normal ground color
                    if (elevation < visualizer::IPV_ELEVATION_BEGIN)
                    {
                        elevation = visualizer::IPV_ELEVATION_BEGIN;
                    }
                    return elevation;
                });
        of.flush();
#if __APPLE__
        std::string strCommand("open " + i_strName);
        auto res = system(strCommand.c_str());
        (void)res;
#endif
    };

} // namespace visualizer

#endif // __VISUALIZER_H__
