#include <pointmatcher/PointMatcher.h>
#include <iostream>

typedef PointMatcher<float> PM;

int main(int argc, char** argv)
{
    if(argc != 3)
    {
        std::cerr << "Incorrect number of arguments! Argument 1 is the input vtk file. Argument 2 is the output csv file." << std::endl;
        exit(1);
    }

    PM::DataPoints cloud = PM::DataPoints::load(argv[1]);
    cloud.save(argv[2]);

    return 0;
}
