#include <iostream>
#include <fstream>
#include <vector>
#include <sstream>
#include <string>
#include "json.hpp"
#include "ObjModel.h"
#include "VoxelGrid.h"
#include "Utilities.h"

// CGAL libraries
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>

using json = nlohmann::json;
using namespace std;

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef K::Point_3 Point3;

int main(int argc, const char * argv[]) {

    // Step 1: read the obj file content into memory
    ObjModel model = readObjFile("../data/IfcOpenHouse_IFC4.obj");


    // Think about removing the slabs bigger than the building footprint


    return 0;
}
