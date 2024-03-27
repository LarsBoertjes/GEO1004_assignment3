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
typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef Kernel::Point_3 Point_3;

struct Object {
    std::string name;
    std::vector<Kernel::Triangle_3> shells;
};

struct Vertex {
    Point_3 point3_origin;
};

int main(int argc, const char *argv[]) {
    std::vector<Vertex> vertices;
    std::vector<int> objects;

    // reading the obj file
    std::ifstream input_stream(input_file);
    if (input_stream.is_open()) {
        std::string line;

        while (getline(input_stream, line)) {
            std::istringstream line_stream(line);
            char line_type;
            line_stream >> line_type;

            if (line_type == 'v') {
                vertices.emplace_back();
                // the coordinates for the vertices are turned into 3d point objects
                double x, y, z;
                line_stream >> x >> y >> z;
                vertices.back().point3_origin = Point_3(x, y, z);
            }

            if (line_type == 'g') {
                objects.emplace_back();
                int v;
                while (!line_stream.eof()) {
                    line_stream >> v;
                    // reference the vertices - turn into point_3
                    // add those point_3 into the triangle_3
                    objects.push_back(v - 1);
                }
            }
        }
    }
}