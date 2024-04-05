//
// Created by ldeni on 3/27/2024.
//

#include <iostream>
#include <fstream>
#include <vector>
#include <sstream>
#include <string>
#include "json.hpp"
#include "ObjModel.h"
#include "voxel_lotte.h"
#include "utilities_lotte.h"
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


    vector<vector<Vertex>> surfaces;

    for (int x = 0; x < nrows_x; ++x) {
        for (int y = 0; y < nrows_y; ++y) {
            for (int z = 0; z < nrows_z; ++z) {
                if (voxelGrid(x, y, z) == 1) {

                    // step 6
                    vector<vector<Vertex>> voxelSurfaces = output_surface(voxelGrid, x, y, z, model);
                    surfaces.insert(surfaces.end(), voxelSurfaces.begin(), voxelSurfaces.end());


                    for (auto surface : voxelSurfaces) {
                        for (auto vertex : surface) {
                            cout << vertex.x << ", " << vertex.y << ", " << vertex.z << endl;
                        }
                    }
                }
            }
        }
    }

    // step 7
    output_to_cityjson(surfaces, "output.city.json");






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






















//#include <iostream>
//#include <fstream>
//#include <sstream>
//#include <vector>
//#include <list>
//#include <map>
//#include <iomanip>
//#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
//#include <CGAL/Constrained_Delaunay_triangulation_2.h>
//#include <CGAL/Triangulation_vertex_base_2.h>
//#include <CGAL/Triangulation_face_base_with_info_2.h>
//#include <CGAL/linear_least_squares_fitting_3.h>
//#include <CGAL/intersections.h>
//#include <stack>
//
//using namespace std;
//typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
//typedef Kernel::Point_3 Point_3;
//typedef Kernel::Triangle_3 Triangle_3;
//typedef Kernel::Line_3 Line_3;
//
//struct Vertex {
//    // maybe Point_3 is better?
//    float x, y, z;
//    Point_3 ptx, pty, ptz;
//    unsigned int x_voxel, y_voxel, z_voxel;
//    float x_model, y_model, z_model;
//    vector<unsigned int> x_vert_vox;
//    vector<unsigned int> y_vert_vox;
//    vector<unsigned int> z_vert_vox;
//};
//
//struct Normal {
//    float n1, n2, n3;
//};
//
//struct Face {
//    vector<int> face_vertex_indices;
//    vector<int> face_normal_indices;
//};
//
//struct Object {
//    string name;
//    vector<Vertex> object_vertices;
//    vector<Normal> object_normals;
//    vector<Face> object_faces;
//};
//
//struct Model {
//    vector<Object> objects;
//    vector<Vertex> model_vertices;
//    vector<Vertex> grid_vertices;
//    float min_x = std::numeric_limits<float>::max(), max_x = std::numeric_limits<float>::lowest();
//    float min_y = std::numeric_limits<float>::max(), max_y = std::numeric_limits<float>::lowest();
//    float min_z = std::numeric_limits<float>::max(), max_z = std::numeric_limits<float>::lowest();
//};
//
//
//struct VoxelGrid {
//    std::vector<unsigned int> voxels;
//    unsigned int max_x, max_y, max_z;
//    float voxel_res;
//
//    VoxelGrid(unsigned int x, unsigned int y, unsigned int z) {
//        max_x = x;
//        max_y = y;
//        max_z = z;
//        unsigned int total_voxels = x * y * z;
//        voxels.reserve(total_voxels);
//        for (unsigned int i = 0; i < total_voxels; ++i) voxels.push_back(0);
//    }
//
//    unsigned int &operator()(const unsigned int &x, const unsigned int &y, const unsigned int &z) {
//        assert(x >= 0 && x < max_x);
//        assert(y >= 0 && y < max_y);
//        assert(z >= 0 && z < max_z);
//        return voxels[x + y * max_x + z * max_x * max_y];
//    }
//
//    unsigned int operator()(const unsigned int &x, const unsigned int &y, const unsigned int &z) const {
//        assert(x >= 0 && x < max_x);
//        assert(y >= 0 && y < max_y);
//        assert(z >= 0 && z < max_z);
//        return voxels[x + y * max_x + z * max_x * max_y];
//    }
//
//    void label_voxels(VoxelGrid &voxel_grid, vector<int> start, int label, unsigned int nrows_x, unsigned int nrows_y,
//                      unsigned int nrows_z) {
//        stack<vector<int>> to_check;
//        to_check.push(start);
//        const std::vector<std::vector<int>> neighbours = {{1,  0,  0},
//                                                          {-1, 0,  0},
//                                                          {0,  1,  0},
//                                                          {0,  -1, 0},
//                                                          {0,  0,  1},
//                                                          {0,  0,  -1}};
//        while (!to_check.empty()) {
//            vector<int> current_vox = to_check.top();
//            to_check.pop();
//            int x = current_vox[0], y = current_vox[1], z = current_vox[2];
//            if (voxel_grid(x, y, z) == 0) {
//                voxel_grid(x, y, z) = label;
//                for (const auto &neighbour: neighbours) {
//                    int nx = x + neighbour[0];
//                    int ny = y + neighbour[1];
//                    int nz = z + neighbour[2];
//                    // Check bounds
//                    if (nx >= 0 && nx < nrows_x && ny >= 0 && ny < nrows_y && nz >= 0 && nz < nrows_z &&
//                        voxel_grid(nx, ny, nz) == 0) {
//                        to_check.push({nx, ny, nz});
//                    }
//                }
//
//            }
//        }
//    };
//
////    void model_to_voxel(Vertex &vertex, float min_x, float min_y, float min_z) const {
////        vertex.x_voxel = static_cast<unsigned int>(std::floor((vertex.x - min_x) / voxel_res)) + 1;
////        vertex.y_voxel = static_cast<unsigned int>(std::floor((vertex.y - min_y) / voxel_res)) + 1;
////        vertex.z_voxel = static_cast<unsigned int>(std::floor((vertex.z - min_z) / voxel_res)) + 1;
////    }
////
////    void voxel_to_model(Vertex& vertex, float min_x, float min_y, float min_z) const {
////        vertex.x_model =  ((vertex.x_voxel + min_x) * voxel_res) - 1;
////        vertex.y_model =  ((vertex.y_voxel + min_y) * voxel_res) - 1;
////        vertex.z_model =  ((vertex.z_voxel + min_z) * voxel_res) -1;
////
////    }
//
//};
//
//float roundtotwo(float num) {
//    return roundf(num * 100) / 100;
//}
//
//int main(int argc, const char *argv[]) {
//    const char *filename = (argc > 1) ? argv[1] : "../data/IfcOpenHouse_IFC4.obj";
//    cout << "Processing: " << filename << endl;
//
//    ifstream input_stream(filename);
//    if (!input_stream.is_open()) {
//        cerr << "Error: Unable to open file." << endl;
//        return 1;
//    }
//
//    // load the obj file into memory
//    string line;
//    Object current_object;
//    Model model;
//    while (getline(input_stream, line)) {
//        istringstream line_stream(line);
//        string line_type;
//        line_stream >> line_type;
//
//        if (line_type == "g") {
//            if (!current_object.name.empty()) {
//                model.objects.push_back(current_object);
//                current_object = Object();
//            }
//            line_stream >> current_object.name;
//        } else if (line_type == "v") {
//            Vertex vertex{};
//            line_stream >> vertex.x >> vertex.y >> vertex.z;
//            current_object.object_vertices.push_back(vertex);
//            model.model_vertices.push_back(vertex);
//        } else if (line_type == "vn") {
//            Normal normal{};
//            line_stream >> normal.n1 >> normal.n2 >> normal.n3;
//            current_object.object_normals.push_back(normal);
//        } else if (line_type == "f") {
//            Face face{};
//            string vertex;
//            while (line_stream >> vertex) {
//                size_t pos1 = vertex.find("//");
//                int vertex_index = stoi(vertex.substr(0, pos1)) - 1;
//                int normal_index = stoi(vertex.substr(pos1 + 2)) - 1;
//                face.face_vertex_indices.push_back(vertex_index);
//                face.face_normal_indices.push_back(normal_index);
//            }
//            current_object.object_faces.push_back(face);
//        }
//    }
//    // creating the triangles
//    vector<Triangle_3> triangles;
//    for (const auto &object : model.objects) {
//        for (const auto &face : object.object_faces) {
//            // check if there are three vertices in the face
//            if (face.face_vertex_indices.size() != 3) {
//                cerr << "Error: The input should be triangulated." << endl;
//            }
//
//            // get the coordinates
//            Vertex vertex1 = model.model_vertices[face.face_vertex_indices[0]];
//            Vertex vertex2 = model.model_vertices[face.face_vertex_indices[1]];
//            Vertex vertex3 = model.model_vertices[face.face_vertex_indices[2]];
//
//            Point_3 p1(vertex1.x, vertex1.y, vertex1.z);
//            Point_3 p2(vertex2.x, vertex2.y, vertex2.z);
//            Point_3 p3(vertex3.x, vertex3.y, vertex3.z);
//            Triangle_3 triangle(p1, p2, p3);
//            triangles.push_back(triangle);
//        }
//    }
//
//
//    // add the last object
//    if (!current_object.name.empty()) {
//        model.objects.push_back(current_object);
//    }
//
//    // The voxel grid
//
//    // get the dimensions of the space
//    float min_x = numeric_limits<float>::max();
//    float max_x = -numeric_limits<float>::max();
//    float min_y = numeric_limits<float>::max();
//    float max_y = -numeric_limits<float>::max();
//    float min_z = numeric_limits<float>::max();
//    float max_z = -numeric_limits<float>::max();
//    for (auto &vertex: model.model_vertices) {
//        if (vertex.x < min_x) min_x = vertex.x;
//        if (vertex.x > max_x) max_x = vertex.x;
//        if (vertex.y < min_y) min_y = vertex.y;
//        if (vertex.y > max_y) max_y = vertex.y;
//        if (vertex.z < min_z) min_z = vertex.z;
//        if (vertex.z > max_z) max_z = vertex.z;
//    }
//
//    float space_dim_x = (max_x - min_x);
//    float space_dim_y = (max_y - min_y);
//    float space_dim_z = (max_z - min_z);
//
//    float resolution = 0.5;
//
//    // calc num of voxels
//    auto rows_x = static_cast<unsigned int>(std::ceil(space_dim_x / resolution)) + 2;
//    auto rows_y = static_cast<unsigned int>(std::ceil(space_dim_y / resolution)) + 2;
//    auto rows_z = static_cast<unsigned int>(std::ceil(space_dim_z / resolution)) + 2;
//
//    VoxelGrid my_building_grid(rows_x, rows_y, rows_z);
//    my_building_grid.voxel_res = resolution;
//    cout<<rows_x<<endl;
//    cout<<rows_y<<endl;
//    cout<<rows_z<<endl;
//
//
//    for (auto triangle : triangles) {
//        // create the bbox of the triangle
//        CGAL::Bbox_3 bbox = triangle.bbox();
//        // iterate over the voxels, starting at the (1,1,1) because of the empty boundary
//        for (unsigned int x = 0; x < rows_x; ++x) {
//            for (unsigned int y = 0; y < rows_y ; ++y) {
//                for (unsigned int z = 0; z < rows_z ; ++z) {
//                    float voxelMinX = min_x + (x - 1) * resolution;
//                    float voxelMinY = min_y + (y - 1) * resolution;
//                    float voxelMinZ = min_z + (z - 1) * resolution;
//                    float voxelMaxX = voxelMinX + resolution;
//                    float voxelMaxY = voxelMinY + resolution;
//                    float voxelMaxZ = voxelMinZ + resolution;
//
//                    CGAL::Bbox_3 voxel_bbox(voxelMinX, voxelMinY, voxelMinZ, voxelMaxX, voxelMaxY, voxelMaxZ);
//
//                    // check if the bounding boxes intersect
//                    if (!CGAL::do_overlap(bbox, voxel_bbox))
//                        continue;
//
//                    // check if the triangle intersects with the voxel bbox
//                    if (CGAL::do_intersect(triangle, voxel_bbox)) {
//                        my_building_grid(x, y, z) = 1;
//                    }
//                }
//            }
//        }
//    }
//
//    //step 5
//    //outside
//    my_building_grid.label_voxels(my_building_grid, {0,  0,  0}, 2, rows_x, rows_y, rows_z );
//    //inside
//    int room_id = 3;
//    for (int x = 0; x < rows_x; ++x) {
//        for (int y = 0; y < rows_y; ++y) {
//            for (int z = 0; z < rows_z; ++z) {
//                if (my_building_grid(x, y, z) == 0) {
//                    my_building_grid.label_voxels(my_building_grid, {x, y, z}, room_id, rows_x, rows_y, rows_z);
//                    room_id++;
//                }
//            }
//        }
//    }
//
////    // my test print statements
////    for (const auto &object: model.objects) {
////        cout << "Object: " << object.name << endl;
////        for (size_t i = 0; i < object.object_faces.size(); ++i) {
////            const Face &face = object.object_faces[i];
////            cout << "Face " << i + 1 << ":\n";
////            cout << "Vertex Coordinates: " << endl;
////            for (int index: face.face_vertex_indices) {
////                Vertex vertex = model.model_vertices[index];
////                cout << "x: " << roundtotwo(vertex.x) << ", y: " << roundtotwo(vertex.y) << ", z: "
////                     << roundtotwo(vertex.z) << endl;
////            }
////            cout << "Normal Indices: ";
////            for (int index: face.face_normal_indices) {
////                cout << index << " ";
////            }
////            cout << endl;
////        }
////    }
//
//    cout << "Voxel Grid Indices:" << endl;
//    for (unsigned int z = 0; z < my_building_grid.max_z; ++z) {
//        cout << z <<endl;
//        for (unsigned int y = 0; y < my_building_grid.max_y; ++y) {
//            for (unsigned int x = 0; x < my_building_grid.max_x; ++x) {
//
//                cout << "(" << my_building_grid(x, y, z) << ") ";
//            }
//            cout << endl;
//        }
//        cout << endl;
//    }
//
////    for (auto &vertex: model.model_vertices) {
////        cout << "Original Vertex: (" << vertex.x << ", " << vertex.y << ", " << vertex.z << ")" << endl;
////        my_building_grid.model_to_voxel(vertex, min_x, min_y, min_z);
////        cout << "Voxel Vertex: (" << vertex.x_voxel << ", " << vertex.y_voxel << ", " << vertex.z_voxel
////             << ")" << endl;
////        // vertex.x etc always store the original, so no need to convert back
////        cout << "Back to Model Vertex: (" << vertex.x << ", " << vertex.y << ", " << vertex.z << ")"
////             << endl;
////    }
//
//
//    return 0;
//};







