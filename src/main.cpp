//
// Created by Lotte de Niet & Lars Boertjes
//

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include "json.hpp"
#include "ObjModel.h"
#include "VoxelGrid.h"
#include "Utilities.h"

// CGAL libraries
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>


typedef CGAL::Simple_cartesian<double> Kernel;
typedef Kernel::Point_3 Point_3;




using json = nlohmann::json;
using namespace std;

typedef K::Point_3 Point3;

int main(int argc, const char *argv[]) {

    // Step 1: From IFC to OBJ
    // -- done using IfcConvert in the commandline

    // Step 2: read the obj file content into memory
    ObjModel model = readObjFile("../data/Wellness_center.obj");

    // Assign bounding box for all groups
    for (auto it = model.groups.begin(); it != model.groups.end(); ) {
        Group& group = *it;
        group.boundingBox = horizontalBoundingBox(group, model);

        if (group.boundingBox > 1200) {
            it = model.groups.erase(it); // Remove the group from the vector
        } else if (group.max_x > 18 || group.min_x < -18 || group.max_y > 9 || group.min_y < -23) {
            it = model.groups.erase(it);
        } else {
            ++it; // Move to the next element
        }
    }

    // Assign min, max values to the object
    assignMinMax(model);

    // Compute bounding box dimensions
    float bbX = model.max_x - model.min_x;
    float bbY = model.max_y - model.min_y;
    float bbZ = model.max_z - model.min_z;

    cout << "Size: " << bbX << " * " << bbY << " * " << bbZ << endl;

    // Step 3: Construct the voxel grid

    // Set the resolution
    float resolution = 0.2;

    // Calculate number of rows in all dimensions + 2 for margin
    int nrows_x = static_cast<unsigned int>(ceil(bbX / resolution)) + 2;
    int nrows_y = static_cast<unsigned int>(ceil(bbY / resolution)) + 2;
    int nrows_z = static_cast<unsigned int>(ceil(bbZ / resolution)) + 2;

    cout << "Using a " << resolution << "m resolution the VoxelGrid will have: " << endl;
    cout << "Number of rows X: " << nrows_x << endl;
    cout << "Number of rows Y: " << nrows_y << endl;
    cout << "Number of rows Z: " << nrows_z << endl;
    cout << "-----------------------" << endl;

    VoxelGrid voxelGrid(nrows_x, nrows_y, nrows_z);
    voxelGrid.resolution = resolution;

    // Translate vertices to integer voxel coordinates
    for (Vertex &vertex: model.vertices) {
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

    // Step 5: Mark the spaces
    // Mark the exterior with id 2
    markSpace(voxelGrid, {0, 0, 0}, 2, nrows_x, nrows_y, nrows_z);

    // Mark the interior rooms with id 3 and higher
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

    // Extract interior
    vector<vector<vector<Vertex>>> allInteriorSurfaces;
    for (int roomId = 3; roomId < room_id; ++roomId) {
        vector<vector<Vertex>> interiorSurfaces;
        for (int x = 0; x < nrows_x; ++x) {
            for (int y = 0; y < nrows_y; ++y) {
                for (int z = 0; z < nrows_z; ++z) {
                    if (voxelGrid(x, y, z) == 1) {
                        vector<vector<Vertex>> voxelSurfaces2 = output_surface(voxelGrid, x, y, z, model,
                                                                                   dilationAmount, roomId);
                        interiorSurfaces.insert(interiorSurfaces.end(), voxelSurfaces2.begin(), voxelSurfaces2.end());
                        }
                    }
                }
            }
        allInteriorSurfaces.push_back(interiorSurfaces);
    }

    // Remove the spaces between walls that are created at a high resolution
    // A room only exists if it's at least 0.5 meter wide
    double minWidth = 0.5;
    double minLength = 0.5;
    double minHeight = 0.8;
    for (int roomId = allInteriorSurfaces.size() - 1; roomId >= 0; --roomId) {
        const auto &roomSurfaces = allInteriorSurfaces[roomId];
        double roomWidth = 0.0;
        double roomLength = 0.0;
        double roomHeight = 0.0;

        // Get the distance between the maximum and minimum for x and y
        vector<double> values = calculatePolygonDimensions(roomSurfaces);
        cout<<values[2]<<endl;
        roomWidth = max(roomWidth, values[0]);
        roomLength = max(roomLength, values[1]);
        roomHeight = max(roomHeight, values[2]);

        // Compare the room size with the threshold. If it's lower, the surface is removed
        if (roomWidth <= minWidth || roomLength <= minLength || roomHeight <= minHeight) {
            allInteriorSurfaces.erase(allInteriorSurfaces.begin() + roomId);
        }
    }


    // Process exterior
    // Interior and exterior have to be processed slightly different because of the rooms
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
    // General settings
    nlohmann::json json;
    json["type"] = "CityJSON";
    json["version"] = "2.0";
    json["transform"] = {{"scale",     {1.0, 1.0, 1.0}},
                         {"translate", {0.0, 0.0, 0.0}}};
    json["CityObjects"] = nlohmann::json::object();

    vector<vector<double>> vertices;

    // Write the exterior
    vector<vector<vector<int>>> ex_boundaries;
    for (const auto &surface: exteriorSurfaces) {
        vector<vector<int>> surfaceBoundaries;
        vector<int> polygonBoundary;
        // Store the vertices, or remember the index if the vertex already exists
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

    // The exterior is added as a Building type
    nlohmann::json cityObject;
    cityObject["type"] = "Building";
    cityObject["geometry"] = nlohmann::json::array();

    cityObject["children"] = nlohmann::json::array();

    for (size_t roomId = 0; roomId < allInteriorSurfaces.size(); ++roomId) {
        // Create a room name and push it into the children array
        cityObject["children"].push_back("BuildingRoom" + std::to_string(roomId + 1));
    }

    nlohmann::json geometryObject = {
            {"type",       "MultiSurface"},
            {"lod",        "3"},
            {"boundaries", ex_boundaries}
    };

    cityObject["geometry"].push_back(geometryObject);
    json["CityObjects"]["Building"] = cityObject;


    // Write the interiors
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

        // The rooms are added as individual BuildingRoom instances
        nlohmann::json cityObject2;
        cityObject2["type"] = "BuildingRoom";
        cityObject2["geometry"] = nlohmann::json::array();
        cityObject2["parents"] = nlohmann::json::array();
        cityObject2["parents"].push_back("Building");
        nlohmann::json geometryObject2 = {
                {"type",       "MultiSurface"},
                {"lod",        "3"},
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

