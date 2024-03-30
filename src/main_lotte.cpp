//
// Created by ldeni on 3/27/2024.
//
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <list>
#include <map>
#include <iomanip>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/Triangulation_vertex_base_2.h>
#include <CGAL/Triangulation_face_base_with_info_2.h>
#include <CGAL/linear_least_squares_fitting_3.h>
#include <CGAL/intersections.h>

using namespace std;
typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef Kernel::Point_3 Point_3;
typedef Kernel::Triangle_3 Triangle_3;
typedef Kernel::Line_3 Line_3;

struct Vertex {
    // maybe Point_3 is better?
    float x, y, z;
    Point_3 ptx, pty, ptz;
    unsigned int x_voxel, y_voxel, z_voxel;
    float x_model, y_model, z_model;
    vector<unsigned int> x_vert_vox;
    vector<unsigned int> y_vert_vox;
    vector<unsigned int> z_vert_vox;
};

struct Normal {
    float n1, n2, n3;
};

struct Face {
    vector<int> face_vertex_indices;
    vector<int> face_normal_indices;
    Triangle_3 triangle;

};

struct Object {
    string name;
    vector<Vertex> object_vertices;
    vector<Normal> object_normals;
    vector<Face> object_faces;
};

struct Model {
    vector<Object> objects;
    vector<Vertex> model_vertices;
};

struct VoxelGrid {
    std::vector<unsigned int> voxels;
    unsigned int max_x, max_y, max_z;
    float voxel_res;

    VoxelGrid(unsigned int x, unsigned int y, unsigned int z) {
        max_x = x;
        max_y = y;
        max_z = z;
        unsigned int total_voxels = x * y * z;
        voxels.reserve(total_voxels);
        for (unsigned int i = 0; i < total_voxels; ++i) voxels.push_back(0);
    }

    unsigned int &operator()(const unsigned int &x, const unsigned int &y, const unsigned int &z) {
        assert(x >= 0 && x < max_x);
        assert(y >= 0 && y < max_y);
        assert(z >= 0 && z < max_z);
        return voxels[x + y * max_x + z * max_x * max_y];
    }

    unsigned int operator()(const unsigned int &x, const unsigned int &y, const unsigned int &z) const {
        assert(x >= 0 && x < max_x);
        assert(y >= 0 && y < max_y);
        assert(z >= 0 && z < max_z);
        return voxels[x + y * max_x + z * max_x * max_y];
    }

    void model_to_voxel(Vertex &vertex, float min_x, float min_y, float min_z) const {
        vertex.x_voxel = static_cast<unsigned int>(std::floor((vertex.x - min_x) / voxel_res)) + 1;
        vertex.y_voxel = static_cast<unsigned int>(std::floor((vertex.y - min_y) / voxel_res)) + 1;
        vertex.z_voxel = static_cast<unsigned int>(std::floor((vertex.z - min_z) / voxel_res)) + 1;
    }

    void voxel_to_model(Vertex& vertex, float min_x, float min_y, float min_z) const {
        vertex.x_model =  ((vertex.x_voxel + min_x) * voxel_res) - 1;
        vertex.y_model =  ((vertex.y_voxel + min_y) * voxel_res) - 1;
        vertex.z_model =  ((vertex.z_voxel + min_z) * voxel_res) -1;

    }

};

float roundtotwo(float num) {
    return roundf(num * 100) / 100;
}

int main(int argc, const char *argv[]) {
    const char *filename = (argc > 1) ? argv[1] : "../data/openhouse.obj";
    cout << "Processing: " << filename << endl;

    ifstream input_stream(filename);
    if (!input_stream.is_open()) {
        cerr << "Error: Unable to open file." << endl;
        return 1;
    }

    // load the obj file into memory
    string line;
    Object current_object;
    Model model;
    while (getline(input_stream, line)) {
        istringstream line_stream(line);
        string line_type;
        line_stream >> line_type;

        if (line_type == "g") {
            if (!current_object.name.empty()) {
                model.objects.push_back(current_object);
                current_object = Object();
            }
            line_stream >> current_object.name;
        } else if (line_type == "v") {
            Vertex vertex{};
            line_stream >> vertex.x >> vertex.y >> vertex.z;
            vertex.ptx = Point_3(vertex.x, 0.0f, 0.0f);
            vertex.pty = Point_3(0.0f, vertex.y, 0.0f);
            vertex.ptz = Point_3(0.0f, 0.0f, vertex.z);
            current_object.object_vertices.push_back(vertex);
            model.model_vertices.push_back(vertex);
        } else if (line_type == "vn") {
            Normal normal{};
            line_stream >> normal.n1 >> normal.n2 >> normal.n3;
            current_object.object_normals.push_back(normal);
        } else if (line_type == "f") {
            Face face{};
            string vertex;
            while (line_stream >> vertex) {
                size_t pos1 = vertex.find("//");
                int vertex_index = stoi(vertex.substr(0, pos1)) - 1;
                int normal_index = stoi(vertex.substr(pos1 + 2)) - 1;
                face.face_vertex_indices.push_back(vertex_index);
                face.face_normal_indices.push_back(normal_index);
            }
            current_object.object_faces.push_back(face);
        }
    }

    // creating the triangles
    vector<Triangle_3> triangles;
    for (const auto &object : model.objects) {
        for (const auto &face : object.object_faces) {
            // check if there are three vertices in the face
            if (face.face_vertex_indices.size() != 3) {
                cerr << "Error: The input should be triangulated." << endl;
            }
            // get the indices
            int index1 = face.face_vertex_indices[0];
            int index2 = face.face_vertex_indices[1];
            int index3 = face.face_vertex_indices[2];

            // get the coordinates
            Vertex vertex1 = model.model_vertices[index1];
            Vertex vertex2 = model.model_vertices[index2];
            Vertex vertex3 = model.model_vertices[index3];

            Point_3 p1(vertex1.x, vertex1.y, vertex1.z);
            Point_3 p2(vertex2.x, vertex2.y, vertex2.z);
            Point_3 p3(vertex3.x, vertex3.y, vertex3.z);
            Triangle_3 triangle(p1, p2, p3);
//            cout << "Triangle: " << triangle << endl;
            triangles.push_back(triangle);
        }
    }



    // add the last object
    if (!current_object.name.empty()) {
        model.objects.push_back(current_object);
    }

    // The voxel grid

    // get the dimensions of the space
    float min_x = std::numeric_limits<float>::max();
    float max_x = -std::numeric_limits<float>::max();
    float min_y = std::numeric_limits<float>::max();
    float max_y = -std::numeric_limits<float>::max();
    float min_z = std::numeric_limits<float>::max();
    float max_z = -std::numeric_limits<float>::max();
    for (auto &vertex: model.model_vertices) {
        if (vertex.x < min_x) min_x = vertex.x;
        if (vertex.x > max_x) max_x = vertex.x;
        if (vertex.y < min_y) min_y = vertex.y;
        if (vertex.y > max_y) max_y = vertex.y;
        if (vertex.z < min_z) min_z = vertex.z;
        if (vertex.z > max_z) max_z = vertex.z;
    }

    float space_dim_x = (max_x - min_x);
    float space_dim_y = (max_y - min_y);
    float space_dim_z = (max_z - min_z);

    float resolution = 0.5;

    // calc num of voxels - resolution 0.5m
    auto rows_x = static_cast<unsigned int>(std::ceil(space_dim_x / resolution)) + 2;
    auto rows_y = static_cast<unsigned int>(std::ceil(space_dim_y / resolution)) + 2;
    auto rows_z = static_cast<unsigned int>(std::ceil(space_dim_z / resolution)) + 2;

    VoxelGrid my_building_grid(rows_x, rows_y, rows_z);

    // allign model to voxel grid
    my_building_grid.voxel_res = resolution;

//    Vertex vertex1;
//    for (auto &vertex: model.model_vertices) {
//        cout << "Original Vertex: (" << vertex.x << ", " << vertex.y << ", " << vertex.z << ")" << endl;
//        my_building_grid.model_to_voxel(vertex, min_x, min_y, min_z);
//
//        cout << "Voxel Vertex: (" << vertex.x_voxel << ", " << vertex.y_voxel << ", " << vertex.z_voxel
//             << ")" << endl;
//
//        float voxelMinX = vertex.x;
//        float voxelMinY = vertex.y;
//        float voxelMinZ = vertex.z;
//        float voxelMaxX = voxelMinX + resolution;
//        float voxelMaxY = voxelMinY + resolution;
//        float voxelMaxZ = voxelMinZ + resolution;
//
//        // Define voxel vertices
//        Point_3 v0(voxelMinX, voxelMinY, voxelMinZ);
//        Point_3 v1(voxelMaxX, voxelMinY, voxelMinZ);
//        Point_3 v2(voxelMaxX, voxelMaxY, voxelMinZ);
//        Point_3 v3(voxelMinX, voxelMaxY, voxelMinZ);
//        Point_3 v4(voxelMinX, voxelMinY, voxelMaxZ);
//        Point_3 v5(voxelMaxX, voxelMinY, voxelMaxZ);
//        Point_3 v6(voxelMaxX, voxelMaxY, voxelMaxZ);
//        Point_3 v7(voxelMinX, voxelMaxY, voxelMaxZ);
//
//        // adds 26-connectivity
//        float voxel_cent_x = vertex.x + (0.5 * resolution);
//        float voxel_cent_y = vertex.y + (0.5 * resolution);
//        float voxel_cent_z = vertex.z + (0.5 * resolution);
//        Line_3 l1(Point_3(voxel_cent_x, vertex.y + resolution, voxel_cent_z),
//                  Point_3(voxel_cent_x, vertex.y, voxel_cent_z));
//        Line_3 l2(Point_3(voxel_cent_x, voxel_cent_y, vertex.z + resolution),
//                  Point_3(voxel_cent_x, voxel_cent_y, vertex.z));
//        Line_3 l3(Point_3(vertex.x + resolution, voxel_cent_y, voxel_cent_z),
//                  Point_3(vertex.x, voxel_cent_y, voxel_cent_z));
//
//        // Define the 12 edges of the voxel
//        Line_3 edges[15] = {
//                Line_3(v0, v1), Line_3(v1, v2), Line_3(v2, v3), Line_3(v3, v0), // Bottom face
//                Line_3(v4, v5), Line_3(v5, v6), Line_3(v6, v7), Line_3(v7, v4), // Top face
//                Line_3(v0, v4), Line_3(v1, v5), Line_3(v2, v6), Line_3(v3, v7),  // Connecting edges
//                l1, l2, l3 //26-connectivity
//        };
//
//        for (auto triangle : triangles) {
//            for (auto edge : edges){
//                if (CGAL::do_intersect(triangle, edge)){
//                    my_building_grid(vertex.x_voxel, vertex.y_voxel, vertex.z_voxel) = 1;
//            }
//        }
//        }
//    }



    for (auto triangle : triangles) {
        cout <<triangle<<endl;
        // iterate over the voxels, starting at the (1,1,1) because of the empty boundary
        for (unsigned int x = 1; x < rows_x - 1; ++x) {
            for (unsigned int y = 1; y < rows_y - 1; ++y) {
                for (unsigned int z = 1; z < rows_z - 1; ++z) {
                    float voxelMinX = min_x + (x - 1) * resolution;
                    float voxelMinY = min_y + (y - 1) * resolution;
                    float voxelMinZ = min_z + (z - 1) * resolution;
                    float voxelMaxX = voxelMinX + resolution;
                    float voxelMaxY = voxelMinY + resolution;
                    float voxelMaxZ = voxelMinZ + resolution;


                    Point_3 v0(voxelMinX, voxelMinY, voxelMinZ);
                    Point_3 v1(voxelMaxX, voxelMinY, voxelMinZ);
                    Point_3 v2(voxelMaxX, voxelMaxY, voxelMinZ);
                    Point_3 v3(voxelMinX, voxelMaxY, voxelMinZ);
                    Point_3 v4(voxelMinX, voxelMinY, voxelMaxZ);
                    Point_3 v5(voxelMaxX, voxelMinY, voxelMaxZ);
                    Point_3 v6(voxelMaxX, voxelMaxY, voxelMaxZ);
                    Point_3 v7(voxelMinX, voxelMaxY, voxelMaxZ);

                    Line_3 edges[12] = {
                            Line_3(v0, v1), Line_3(v1, v2), Line_3(v2, v3), Line_3(v3, v0), // Bottom face
                            Line_3(v4, v5), Line_3(v5, v6), Line_3(v6, v7), Line_3(v7, v4), // Top face
                            Line_3(v0, v4), Line_3(v1, v5), Line_3(v2, v6), Line_3(v3, v7)  // Connecting edges
                    };



                    for (auto edge : edges) {
                        if (CGAL::do_intersect(triangle, edge)) {
                            my_building_grid(x, y, z) = 1;
                            break;
                        }
                    }
                }
            }
        }
    }


//    // my test print statements
//    for (const auto &object: model.objects) {
//        cout << "Object: " << object.name << endl;
//        for (size_t i = 0; i < object.object_faces.size(); ++i) {
//            const Face &face = object.object_faces[i];
//            cout << "Face " << i + 1 << ":\n";
//            cout << "Vertex Coordinates: " << endl;
//            for (int index: face.face_vertex_indices) {
//                Vertex vertex = model.model_vertices[index];
//                cout << "x: " << roundtotwo(vertex.x) << ", y: " << roundtotwo(vertex.y) << ", z: "
//                     << roundtotwo(vertex.z) << endl;
//            }
//            cout << "Normal Indices: ";
//            for (int index: face.face_normal_indices) {
//                cout << index << " ";
//            }
//            cout << endl;
//        }
//    }

    cout << "Voxel Grid Indices:" << endl;
    for (unsigned int z = 0; z < my_building_grid.max_z; ++z) {
        for (unsigned int y = 0; y < my_building_grid.max_y; ++y) {
            for (unsigned int x = 0; x < my_building_grid.max_x; ++x) {
                cout << "(" << my_building_grid(x, y, z) << ") ";
            }
            cout << endl;
        }
        cout << endl;
    }

//    for (auto &vertex: model.model_vertices) {
//        cout << "Original Vertex: (" << vertex.x << ", " << vertex.y << ", " << vertex.z << ")" << endl;
//        my_building_grid.model_to_voxel(vertex, min_x, min_y, min_z);
//        cout << "Voxel Vertex: (" << vertex.x_voxel << ", " << vertex.y_voxel << ", " << vertex.z_voxel
//             << ")" << endl;
//        // vertex.x etc always store the original, so no need to convert back
//        cout << "Back to Model Vertex: (" << vertex.x << ", " << vertex.y << ", " << vertex.z << ")"
//             << endl;
//    }


    return 0;
};