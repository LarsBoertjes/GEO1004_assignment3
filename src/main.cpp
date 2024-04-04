#include <iostream>
#include <fstream>
#include <vector>
#include <sstream>
#include <string>
#include "json.hpp"
#include "ObjModel.h"
#include "VoxelGrid.h"
#include "Utilities.h"
#include <queue>

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

    cout << "Size: " << bbX << " * " << bbY << " * " << bbZ << endl;

    // Step 3: Construct the voxel grid
    // Set the resolution
    float resolution = 0.5;

    // Calculate number of rows in all dimensions + 2 for margin
    int nrows_x = static_cast<unsigned int>(std::ceil(bbX / resolution)) + 2;
    int nrows_y = static_cast<unsigned int>(std::ceil(bbY / resolution)) + 2;
    int nrows_z = static_cast<unsigned int>(std::ceil(bbZ / resolution)) + 2;

    cout << "Using a " << resolution << "m resolution the VoxelGrid will have: " << endl;
    cout << "Number of rows X: " << nrows_x << endl;
    cout << "Number of rows Y: " << nrows_y << endl;
    cout << "Number of rows Z: " << nrows_z << endl;
    cout << "-----------------------" << endl;

    VoxelGrid voxelGrid(nrows_x, nrows_y, nrows_z);
    voxelGrid.resolution = resolution;

    // Translate vertices to integer voxel coordinates
    for (Vertex &vertex : model.vertices) {
        model.model_vertices.push_back(voxelGrid.model_to_voxel(vertex, model.min_x, model.min_y, model.min_z));
    }

    // Step 4: Voxelise the grid
    // Get the triangle faces from the model
    vector<Triangle3> trianglesModelCoordinates = extractTriangles(model).first;
    vector<Triangle3> trianglesGridCoordinates = extractTriangles(model).second;

    cout << "Number of triangles to test for intersection: " << trianglesModelCoordinates.size() << endl;
    cout << "Number of triangles to test for intersection: " << trianglesGridCoordinates.size() << endl;

    // Checking triangle intersection with VoxelGrid
    markGrid(trianglesModelCoordinates, trianglesGridCoordinates, model, voxelGrid);

    // Mark the spaces
    markSpace(voxelGrid, {0, 0, 0}, 2, nrows_x, nrows_y, nrows_z);

    int room_id = 3;
    for (int x = 1; x < nrows_x - 1; ++x) {
        for (int y = 1; y < nrows_y - 1; ++y) {
            for (int z = 1; z < nrows_z - 1; ++z) {
                if (voxelGrid(x, y, z) == 0) {
                    markSpace(voxelGrid, {x, y, z}, room_id, nrows_x, nrows_y, nrows_z);
                    room_id++;
                }
            }
        }
    }

    // Testing to print layers
    for (int z = 0; z < nrows_z; ++z) {
        cout << "z: " << z << endl;
        for (int j = 0; j < nrows_y; ++j) {
            for (int i = 0; i < nrows_x; ++i) {
                unsigned int v = voxelGrid(i, j, z);
                cout << v << " ";
            }
            cout << endl;
        }
        cout << endl;
        cout << "---------------" << endl;
    }

    return 0;
}
