#ifndef GEO1004_ASSIGNMENT3_UTILITIES_H
#define GEO1004_ASSIGNMENT3_UTILITIES_H

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include "ObjModel.h"
#include <string>

using namespace std;

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef K::Segment_3 Segment3;
typedef K::Triangle_3 Triangle3;
typedef K::Point_3 Point3;

ObjModel readObjFile(const std::string& filePath);
void printModelInfo(const ObjModel& model, bool printGroupDetails);
bool doesIntersect(const Triangle& triangle, const VoxelGrid& grid, unsigned int x, unsigned int y, unsigned int z);
void voxeliseModel(ObjModel& model, VoxelGrid& voxelGrid);

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

bool doesIntersect(const Triangle3& triangle, const VoxelGrid& grid, unsigned int x, unsigned int y, unsigned int z) {
    // Define voxel corners in model coordinates
    float voxelMinX = x * grid.resolution;
    float voxelMinY = y * grid.resolution;
    float voxelMinZ = z * grid.resolution;
    float voxelMaxX = voxelMinX + grid.resolution;
    float voxelMaxY = voxelMinY + grid.resolution;
    float voxelMaxZ = voxelMinZ + grid.resolution;

    // Define voxel vertices
    Point3 v0(voxelMinX, voxelMinY, voxelMinZ);
    Point3 v1(voxelMaxX, voxelMinY, voxelMinZ);
    Point3 v2(voxelMaxX, voxelMaxY, voxelMinZ);
    Point3 v3(voxelMinX, voxelMaxY, voxelMinZ);
    Point3 v4(voxelMinX, voxelMinY, voxelMaxZ);
    Point3 v5(voxelMaxX, voxelMinY, voxelMaxZ);
    Point3 v6(voxelMaxX, voxelMaxY, voxelMaxZ);
    Point3 v7(voxelMinX, voxelMaxY, voxelMaxZ);

    // Define the 12 edges of the voxel
    Segment3 edges[12] = {
            Segment3(v0, v1), Segment3(v1, v2), Segment3(v2, v3), Segment3(v3, v0), // Bottom face
            Segment3(v4, v5), Segment3(v5, v6), Segment3(v6, v7), Segment3(v7, v4), // Top face
            Segment3(v0, v4), Segment3(v1, v5), Segment3(v2, v6), Segment3(v3, v7)  // Connecting edges
    };

    for (auto& edge : edges) {
        if (CGAL::do_intersect(triangle, edge)) {
            return true;
        }
    }
    return false;
}


void voxeliseModel(ObjModel& model, VoxelGrid& voxelGrid) {
    for (const auto& group : model.groups) {
        for (const auto& face : group.groupFaces) {
            // Populate triangle.vertices from face.vertexIndices and model.vertices
            Point3 v0(model.model_vertices[face.vertexIndices[0]].x, model.model_vertices[face.vertexIndices[0]].y, model.model_vertices[face.vertexIndices[0]].z);
            Point3 v1(model.model_vertices[face.vertexIndices[1]].x, model.model_vertices[face.vertexIndices[1]].y, model.model_vertices[face.vertexIndices[1]].z);
            Point3 v2(model.model_vertices[face.vertexIndices[2]].x, model.model_vertices[face.vertexIndices[2]].y, model.model_vertices[face.vertexIndices[2]].z);

            // Calculate bounding box in voxel grid coordinates
            unsigned int minX = std::min({v0.x(), v1.x(), v2.x()});
            unsigned int minY = std::min({v0.y(), v1.y(), v2.y()});
            unsigned int minZ = std::min({v0.z(), v1.z(), v2.z()});
            unsigned int maxX = std::max({v0.x(), v1.x(), v2.x()});
            unsigned int maxY = std::max({v0.y(), v1.y(), v2.y()});
            unsigned int maxZ = std::max({v0.z(), v1.z(), v2.z()});

            Triangle3 triangle(v0, v1, v2);

            for (unsigned int x = minX; x <= maxX; ++x) {
                for (unsigned int y = minY; y <= maxY; ++y) {
                    for (unsigned int z = minZ; z <= maxZ; ++z) {
                        if (doesIntersect(triangle, voxelGrid, x, y, z)) {
                            voxelGrid(x, y, z) = 1; // Mark voxel as occupied
                        }
                    }
                }
            }
        }
    }
}

#endif //GEO1004_ASSIGNMENT3_UTILITIES_H


