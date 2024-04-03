#ifndef GEO1004_ASSIGNMENT3_UTILITIES_H
#define GEO1004_ASSIGNMENT3_UTILITIES_H

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include "ObjModel.h"
#include <string>
#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_traits.h>
#include <CGAL/AABB_triangle_primitive.h>
#include <vector>

using namespace std;

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef K::Triangle_3 Triangle3;
typedef K::Point_3 Point3;

ObjModel readObjFile(const std::string& filePath);
void printModelInfo(const ObjModel& model, bool printGroupDetails);
std::pair<std::vector<Triangle3>, std::vector<Triangle3>> extractTriangles(const ObjModel& model);
void markGrid(const vector<Triangle3> &trianglesModel, const vector<Triangle3> &trianglesGrid,
              const ObjModel &model, VoxelGrid& voxelGrid);

ObjModel readObjFile(const string& filePath) {
    // Read in ObjFile to Struct:
    // Groups consisting of faces
    // Vertices for the whole model
    // Min/max dimensions

    ObjModel model;
    ifstream objFile(filePath);
    Group currentGroup;
    string line;

    if (objFile.is_open()) {
        while (getline(objFile, line)) {
            stringstream ss(line);
            string prefix;
            ss >> prefix;

            if (prefix == "v") {
                Vertex v;
                ss >> v.x >> v.y >> v.z;
                model.vertices.push_back(v);
            } else if (prefix == "vn") {
                Normal n;
                ss >> n.nx >> n.ny >> n.nz;
                model.normals.push_back(n);
            } else if (prefix == "f") {
                Face f;
                string vertex;
                while (ss >> vertex) {
                    size_t pos1 = vertex.find("//");
                    int vIndex = stoi(vertex.substr(0, pos1)) - 1;
                    int nIndex = stoi(vertex.substr(pos1 + 2)) - 1;
                    f.vertexIndices.push_back(vIndex);
                    f.normalIndices.push_back(nIndex);
                }
                currentGroup.groupFaces.push_back(f);
            } else if (prefix == "g") {
                if (!currentGroup.groupFaces.empty()) {
                    model.groups.push_back(currentGroup);
                    currentGroup = Group();
                }
                ss >> currentGroup.groupname;
            } else if (prefix == "s") {
                string smoothing;
                ss >> smoothing;
                if (smoothing == "off") {
                    currentGroup.smoothingParameter = 0;
                } else {
                    currentGroup.smoothingParameter = stoi(smoothing);
                }
            } else if (prefix == "usemtl") {
                ss >> currentGroup.usemtl;
            }
        }
        if (!currentGroup.groupFaces.empty()) {
            model.groups.push_back(currentGroup);
        }
        objFile.close();
    } else {
        cout << "Unable to open file" << endl;
    }

    cout << "Successfully read " << filePath << " in memory with following content: " << endl;
    printModelInfo(model, false);

    return model;
}

void printModelInfo(const ObjModel& model, bool printGroupDetails) {
    // Print the ObjModel Information
    // Boolean to enable/disable printing of group names

    cout << "Number of vertices: " << model.vertices.size() << endl;
    cout << "Number of normals: " << model.normals.size() << endl;
    cout << "Number of groups: " << model.groups.size() << endl;

    int totalFaces = 0;
    for (const auto& group : model.groups) {
        if (printGroupDetails) {
            cout << "Group: " << group.groupname << ", Faces: " << group.groupFaces.size();
            cout << ", Smoothing: " << group.smoothingParameter << ", Material: " << group.usemtl << endl;
        }
        totalFaces += group.groupFaces.size();
    }

    cout << "Total number of faces: " << totalFaces << endl;
    cout << "-----------------------" << endl;
}

void assignMinMax(ObjModel& model) {
    // Assigns min, max values for all three directions to ObjModel
    // Iterate over all the vertices of the ObjModel and update if needed
    for (const Vertex &vertex : model.vertices) {
        if (vertex.x < model.min_x) model.min_x = vertex.x;
        if (vertex.x > model.max_x) model.max_x = vertex.x;
        if (vertex.y < model.min_y) model.min_y = vertex.y;
        if (vertex.y > model.max_y) model.max_y = vertex.y;
        if (vertex.z < model.min_z) model.min_z = vertex.z;
        if (vertex.z > model.max_z) model.max_z = vertex.z;
    }
}

std::pair<std::vector<Triangle3>, std::vector<Triangle3>> extractTriangles(const ObjModel& model) {
    vector<Triangle3> trianglesModelCoordinates;
    vector<Triangle3> trianglesGridCoordinates;

    for (const auto &group : model.groups) {
        for (const auto &face : group.groupFaces) {
            // Check if the face is a triangle
            if (face.vertexIndices.size() != 3) {
                cerr << "Error: The input should be triangulated." << endl;
            }
            // Get the object coordinates
            Vertex v1 = model.vertices[face.vertexIndices[0]];
            Vertex v2 = model.vertices[face.vertexIndices[1]];
            Vertex v3 = model.vertices[face.vertexIndices[2]];

            // Construct the triangle
            Point3 p1(v1.x, v1.y, v1.z);
            Point3 p2(v2.x, v2.y, v2.z);
            Point3 p3(v3.x, v3.y, v3.z);
            Triangle3 triangle(p1, p2, p3);
            trianglesModelCoordinates.push_back(triangle);

            // Get the grid coordinates
            Vertex vg1 = model.model_vertices[face.vertexIndices[0]];
            Vertex vg2 = model.model_vertices[face.vertexIndices[1]];
            Vertex vg3 = model.model_vertices[face.vertexIndices[2]];

            // Construct the triangle
            Point3 pg1(vg1.x, vg1.y, vg1.z);
            Point3 pg2(vg2.x, vg2.y, vg2.z);
            Point3 pg3(vg3.x, vg3.y, vg3.z);
            Triangle3 triangleGrid(pg1, pg2, pg3);
            trianglesGridCoordinates.push_back(triangleGrid);
        }
    }

    return std::make_pair(trianglesModelCoordinates, trianglesGridCoordinates);
}

void markGrid(const vector<Triangle3> &trianglesModel, const vector<Triangle3> &trianglesGrid,
              const ObjModel &model, VoxelGrid& voxelGrid) {
    // Iterate over all triangles and check with which voxels they overlap
    // Mark the voxel als occupied if intersects

    for (int i = 0; i < trianglesModel.size(); ++i) {
        CGAL::Bbox_3 bboxTriangleModel = trianglesModel[i].bbox();
        CGAL::Bbox_3 bboxTriangleGrid = trianglesGrid[i].bbox();

        for (int x = bboxTriangleGrid.xmin(); x < bboxTriangleGrid.xmax() + 1; ++x) {
            for (int y = bboxTriangleGrid.ymin(); y < bboxTriangleGrid.ymax() + 1; ++y) {
                for (int z = bboxTriangleGrid.zmin(); z < bboxTriangleGrid.zmax() + 1; ++z) {
                    float resolution = voxelGrid.resolution;

                    float voxelMinX = model.min_x + x * resolution;
                    float voxelMinY = model.min_y + y * resolution;
                    float voxelMinZ = model.min_z + z * resolution;
                    float voxelMaxX = voxelMinX + resolution;
                    float voxelMaxY = voxelMinY + resolution;
                    float voxelMaxZ = voxelMinZ + resolution;

                    CGAL::Bbox_3 voxel_bbox(voxelMinX, voxelMinY, voxelMinZ, voxelMaxX, voxelMaxY, voxelMaxZ);

                    if (CGAL::do_intersect(trianglesModel[i], voxel_bbox)) {
                        voxelGrid(x + 1, y + 1, z + 1) = 1;
                    }
                }
            }
        }
    }
}

#endif //GEO1004_ASSIGNMENT3_UTILITIES_H


