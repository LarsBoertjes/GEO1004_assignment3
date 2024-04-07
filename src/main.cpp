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
    ObjModel model = readObjFile("../data/2022020320211122Wellness.obj");

    // Assign min, max values to the object
    assignMinMax(model);

    vector<float> boundingBoxes;

    // Assign bounding box for all groups
    for (auto it = model.groups.begin(); it != model.groups.end(); ) {
        Group& group = *it;
        group.boundingBox = horizontalBoundingBox(group, model);

        if (group.boundingBox > 1000) {
            it = model.groups.erase(it); // Remove the group from the vector
        } else if (group.groupname == "df") {
            it = model.groups.erase(it);
        } else {
                ++it; // Move to the next element
            }
        boundingBoxes.push_back(group.boundingBox);
    }

    cout << "Number of groups: " << model.groups.size() << endl;
    cout << "Number of bboxes: " << boundingBoxes.size() << endl;

    std::sort(boundingBoxes.begin(), boundingBoxes.end(), std::greater<float>());

    std::cout << "Top103 biggest bounding boxes:" << std::endl;
    for (int i = 0; i < 10 && i < boundingBoxes.size(); ++i) {
        std::cout << "BoundingBox " << i + 1 << ": " << boundingBoxes[i] << std::endl;
    }

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

    // Step 6: Extract the surfaces
    // We need to dilate the surfaces to fill the empty space

    float dilationAmount = (1 - resolution);

    // Process interior
    vector<vector<vector<Vertex>>> allInteriorSurfaces;
    for (int roomId = 3; roomId < room_id; ++roomId) {
        vector<vector<Vertex>> interior_Surfaces;
        for (int x = 0; x < nrows_x; ++x) {
            for (int y = 0; y < nrows_y; ++y) {
                for (int z = 0; z < nrows_z; ++z) {
                    if (voxelGrid(x, y, z) == 1) {
                        vector<vector<Vertex>> voxelSurfaces2 = output_int_surface(voxelGrid, x, y, z, model,
                                                                                   dilationAmount, roomId);
                        interior_Surfaces.insert(interior_Surfaces.end(), voxelSurfaces2.begin(),
                                                 voxelSurfaces2.end());
                    }
                }
            }
        }
        allInteriorSurfaces.push_back(interior_Surfaces);
    }

    // Process exterior
    // Interior and exterior have to be processed slightly different
    vector<vector<Vertex>> exteriorSurfaces;
    for (int x = 0; x < nrows_x; ++x) {
        for (int y = 0; y < nrows_y; ++y) {
            for (int z = 0; z < nrows_z; ++z) {
                if (voxelGrid(x, y, z) == 1) {
                    vector<vector<Vertex>> voxelSurfaces = output_surface(voxelGrid, x, y, z, model,
                                                                          dilationAmount, 2);
                    exteriorSurfaces.insert(exteriorSurfaces.end(), voxelSurfaces.begin(), voxelSurfaces.end());
                }
            }
        }
    }

    // Step 7: Output to CityJSON
    nlohmann::json json;
    json["type"] = "CityJSON";
    json["version"] = "2.0";
    json["transform"] = {{"scale",     {1.0, 1.0, 1.0}},
                         {"translate", {0.0, 0.0, 0.0}}};
    json["CityObjects"] = nlohmann::json::object();

    vector<vector<double>> vertices;

    // write the exterior
    vector<vector<vector<int>>> ex_boundaries;
    for (const auto &surface: exteriorSurfaces) {
        vector<vector<int>> surfaceBoundaries;
        vector<int> polygonBoundary;
        for (const auto &vertex: surface) {
            auto it = find(vertices.begin(), vertices.end(), vector<double>{vertex.x, vertex.y, vertex.z});

            if (it == vertices.end()) {
                vertices.push_back({vertex.x, vertex.y, vertex.z});
                polygonBoundary.push_back(vertices.size() - 1);
            } else {
                polygonBoundary.push_back(distance(vertices.begin(), it));
            }
        }

        surfaceBoundaries.push_back(polygonBoundary);
        ex_boundaries.push_back(surfaceBoundaries);
    }

    nlohmann::json cityObject;
    cityObject["type"] = "Building";
    cityObject["geometry"] = nlohmann::json::array();

    nlohmann::json geometryObject = {
            {"type",       "MultiSurface"},
            {"lod",        "2"},
            {"boundaries", ex_boundaries}
    };

    cityObject["geometry"].push_back(geometryObject);
    json["CityObjects"]["Building"] = cityObject;


    // write the interiors
    for (size_t roomId = 0; roomId < allInteriorSurfaces.size(); ++roomId) {
        vector<vector<Vertex>> &interiorSurfaces = allInteriorSurfaces[roomId];
        vector<vector<vector<int>>> int_boundaries;

        for (const auto &surface: interiorSurfaces) {
            vector<vector<int>> surfaceBoundaries;
            vector<int> polygonBoundary;
            for (const auto &vertex: surface) {
                auto it = find(vertices.begin(), vertices.end(), vector<double>{vertex.x, vertex.y, vertex.z});

                if (it == vertices.end()) {
                    vertices.push_back({vertex.x, vertex.y, vertex.z});
                    polygonBoundary.push_back(vertices.size() - 1);
                } else {
                    polygonBoundary.push_back(distance(vertices.begin(), it));
                }
            }

            surfaceBoundaries.push_back(polygonBoundary);
            int_boundaries.push_back(surfaceBoundaries);
        }

        nlohmann::json cityObject2;
        cityObject2["type"] = "BuildingRoom";
        cityObject2["geometry"] = nlohmann::json::array();
        nlohmann::json geometryObject2 = {
                {"type",       "MultiSurface"},
                {"lod",        "2"},
                {"boundaries", int_boundaries}
        };

        cityObject2["geometry"].push_back(geometryObject2);
        json["CityObjects"]["BuildingRoom" + to_string(roomId + 1)] = cityObject2;
    }

    json["vertices"] = vertices;

    // Write JSON to file
    ofstream out_stream("output.city.json");
    out_stream << setw(4) << json;
    out_stream.close();

    return 0;
}
