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
using namespace std;
typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef Kernel::Point_3 Point_3;


struct Vertex {
    // maybe Point_3 is better?
    float x, y, z;
};

struct Normal {
    float n1, n2, n3;
};

struct Face{
    // and maybe save the triangle_3 / triangulation
    vector<int> face_vertex_indices;
    vector<int> face_normal_indices;
};

struct Object {
    string name;
    vector<Vertex> object_vertices;
    vector<Normal> object_normals;
    vector<Face> object_faces;
};

struct Model {
    vector<Object> objects;
};



int main(int argc, const char *argv[]) {
    const char *filename = (argc > 1) ? argv[1] : "../data/out2.obj";
    cout << "Processing: " << filename << endl;

    ifstream input_stream(filename);
    if (!input_stream.is_open()) {
        cerr << "Error: Unable to open file." << endl;
        return 1;
    }

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
            current_object.object_vertices.push_back(vertex);
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
    // add the last object
    if (!current_object.name.empty()) {
        model.objects.push_back(current_object);
    }

    for (const auto &object : model.objects) {
        cout << "Object: " << object.name << endl;
        for (size_t i = 0; i < object.object_faces.size(); ++i) {
            const Face &face = object.object_faces[i];
            cout << "Face " << i + 1 << ":\n";
            cout << "Vertex Indices: ";
            for (int index : face.face_vertex_indices) {
                cout << index << " ";
            }
            cout << "\nNormal Indices: ";
            for (int index : face.face_normal_indices) {
                cout << index << " ";
            }
            cout << endl;
        }
    }

    return 0;
}