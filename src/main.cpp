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
    // Set the resolution
    float resolution = 0.5;

    // Calculate number of rows in all dimensions + 2 for margin
    int num_rows_x = static_cast<unsigned int>(std::ceil(bbX / resolution)) + 2;
    int num_rows_y = static_cast<unsigned int>(std::ceil(bbY / resolution)) + 2;
    int num_rows_z = static_cast<unsigned int>(std::ceil(bbZ / resolution)) + 2;

    cout << "Using a " << resolution << "m resolution the VoxelGrid will have: " << endl;
    cout << "Number of rows X: " << num_rows_x << endl;
    cout << "Number of rows Y: " << num_rows_y << endl;
    cout << "Number of rows Z: " << num_rows_z << endl;
    cout << "-----------------------" << endl;

    VoxelGrid voxelGrid(num_rows_x, num_rows_y, num_rows_z);
    voxelGrid.resolution = resolution;

    // Translate vertices to integer voxel coordinates
    for (Vertex &vertex : model.vertices) {
        model.model_vertices.push_back(voxelGrid.model_to_voxel(vertex, model.min_x, model.min_y, model.min_z));
    }

    // Step 4: Voxelise the grid
    // I want to use the functions here!
    voxeliseModel(model, voxelGrid);


    // Testing to print first layer
    cout << "Printing grid layer z = 0 " << endl;
    for (int i = 0; i < num_rows_x; ++i) {
        for (int j = 0; j < num_rows_y; ++j) {
            unsigned int v = voxelGrid(i, j, 0);
            cout << v << " ";
        }
        cout << endl;
    }

    cout << "-----------------------" << endl;





    return 0;
}
