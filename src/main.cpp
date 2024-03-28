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

    // Step 1: From IFC to OBJ
    // -- done using IfcConvert in the commandline

    // Step 2: read the obj file content into memory
    ObjModel model = readObjFile("../data/IfcOpenHouse_IFC4.obj");

    // Assign min, max values to the object
    assignMinMax(model);

    // Compute bounding box dimensions
    float bbX = model.max_x - model.min_x;
    float bbY = model.max_y - model.min_y;
    float bbZ = model.max_z - model.min_z;

    // Step 3: Construct the voxel grid
    // Initialize resolution
    float resolution = 0.5;

    // Calculate number of rows in all dimensions
    int num_rows_x = static_cast<unsigned int>(std::ceil(bbX / 0.5f));
    int num_rows_y = static_cast<unsigned int>(std::ceil(bbY / 0.5f));
    int num_rows_z = static_cast<unsigned int>(std::ceil(bbZ / 0.5f));

    cout << "Using a " << resolution << "m resolution the VoxelGrid will have: " << endl;
    cout << "Number of rows X: " << num_rows_x << endl;
    cout << "Number of rows Y: " << num_rows_y << endl;
    cout << "Number of rows Z: " << num_rows_z << endl;

    VoxelGrid voxelGrid(num_rows_x, num_rows_y, num_rows_z);

    // Step 4: Voxelisation of triangles
    // still to do



    return 0;
}
