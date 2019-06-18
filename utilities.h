//
// Created by Franz Pucher on 2019-05-22.
//

#ifndef BACHELORTEST_UTILITIES_H
#define BACHELORTEST_UTILITIES_H


#include <fstream>
#include <string>
#include <vector>
#include <exception>




#include "planner.h"
#include "audi_rover.h"

#include "location.h"

using namespace planner;

#ifdef _MSC_VER
static const char* PATH_SEP = "\\";
#else
static const char* PATH_SEP = "/";
#endif

#include "constants.h"

std::ifstream::pos_type fileSize(const std::string& filename)
{
    std::ifstream in(filename, std::ifstream::ate | std::ifstream::binary);
    if (!in.good())
    {
        throw std::exception();
    }
    return in.tellg();
}

std::vector<uint8_t> loadFile(const std::string& filename, size_t expectedFileSize)
{
    size_t fsize = fileSize(filename);
    if (fsize != expectedFileSize)
    {
        throw std::exception();
    }
    std::vector<uint8_t> data(fsize);
    std::ifstream ifile(filename, std::ifstream::binary);
    if (!ifile.good())
    {
        throw std::exception();
    }
    ifile.read((char*)&data[0], fsize);
    return data;
}

void writeFile(const std::string& filename, std::vector<uint8_t> &data, size_t expectedFileSize)
{
    std::ofstream ofile(filename, std::ofstream::binary);

    if (!ofile.good())
    {
        throw std::exception();
    }
    ofile.write((char*)&data[0], expectedFileSize);

    ofile.close();
}


#endif //BACHELORTEST_UTILITIES_H
