#ifndef GEO1004_ASSIGNMENT3_UTILITIES_H
#define GEO1004_ASSIGNMENT3_UTILITIES_H

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include "ObjModel.h"
#include <string>
#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_traits.h>
#include <CGAL/AABB_triangle_primitive.h>
#include <vector>
#include <stack>

using namespace std;

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef K::Triangle_3 Triangle3;
typedef K::Point_3 Point3;

ObjModel readObjFile(const string &filePath);

void printModelInfo(const ObjModel &model, bool printGroupDetails);

pair<vector<Triangle3>, vector<Triangle3>> extractTriangles(const ObjModel &model);

void markGrid(const vector<Triangle3> &trianglesModel, const vector<Triangle3> &trianglesGrid,
              const ObjModel &model, VoxelGrid &voxelGrid);

ObjModel readObjFile(const string &filePath) {
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


float horizontalBoundingBox(Group &group, ObjModel &model) {
    // Constructs the bounding box of the model
    for (const Face &face: group.groupFaces) {
        for (int vertex: face.vertexIndices) {
            if (model.vertices[vertex].x < group.min_x) group.min_x = model.vertices[vertex].x;
            if (model.vertices[vertex].x > group.max_x) group.max_x = model.vertices[vertex].x;
            if (model.vertices[vertex].y < group.min_y) group.min_y = model.vertices[vertex].y;
            if (model.vertices[vertex].y > group.max_y) group.max_y = model.vertices[vertex].y;
            if (model.vertices[vertex].z < group.min_z) group.min_z = model.vertices[vertex].z;
            if (model.vertices[vertex].z > group.max_z) group.max_z = model.vertices[vertex].z;
        }
    }

    float boundingBox = ((group.max_x - group.min_x) * (group.max_y - group.min_y));

    return boundingBox;
}

bool direction_true(const VoxelGrid &grid, int x, int y, int z, string direction, int label) {
    // Returns true for each side of the voxel that has the chosen label
    if (direction == "front") {
        return grid(x, y + 1, z) == label;
    }
    if (direction == "back") {
        return grid(x, y - 1, z) == label;
    }
    if (direction == "right") {
        return grid(x + 1, y, z) == label;
    }
    if (direction == "left") {
        return grid(x - 1, y, z) == label;
    }
    if (direction == "up") {
        return grid(x, y, z + 1) == label;
    }
    if (direction == "down") {
        return grid(x, y, z - 1) == label;
    }

    return false;
}

vector<vector<Vertex>>
output_surface(VoxelGrid &grid, int x, int y, int z, ObjModel &model, float dilationAmount, int side) {
    // Constructs the surfaces in model coordinates
    vector<vector<Vertex>> outputSurfaces;

    // The dilationAmount makes the voxels bigger and touch each other again
    // e.g. From voxel size 0.2 (spaced 0.8 apart) to 1 (because of dilation) to 0.2 (adjacent and correct scale)
    float voxelMinX = (model.min_x + x - 1 * grid.resolution) - dilationAmount;
    float voxelMinY = (model.min_y + y - 1 * grid.resolution) - dilationAmount;
    float voxelMinZ = (model.min_z + z - 1 * grid.resolution) - dilationAmount;
    float voxelMaxX = (voxelMinX + grid.resolution) + dilationAmount;
    float voxelMaxY = (voxelMinY + grid.resolution) + dilationAmount;
    float voxelMaxZ = (voxelMinZ + grid.resolution) + dilationAmount;

    // Scale the touching voxels back
    voxelMinX *= grid.resolution;
    voxelMinY *= grid.resolution;
    voxelMinZ *= grid.resolution;
    voxelMaxX *= grid.resolution;
    voxelMaxY *= grid.resolution;
    voxelMaxZ *= grid.resolution;

    // For each direction that is facing a room or the exterior, a surface is constructed counterclockwise
    if (direction_true(grid, x, y, z, "front", side)) {
        vector<Vertex> frontSurface = {
                {voxelMaxX, voxelMaxY, voxelMinZ},
                {voxelMinX, voxelMaxY, voxelMinZ},
                {voxelMinX, voxelMaxY, voxelMaxZ},
                {voxelMaxX, voxelMaxY, voxelMaxZ}
        };
        outputSurfaces.push_back(frontSurface);
    }
    if (direction_true(grid, x, y, z, "back", side)) {
        vector<Vertex> backSurface = {
                {voxelMinX, voxelMinY, voxelMinZ},
                {voxelMaxX, voxelMinY, voxelMinZ},
                {voxelMaxX, voxelMinY, voxelMaxZ},
                {voxelMinX, voxelMinY, voxelMaxZ}
        };
        outputSurfaces.push_back(backSurface);
    }
    if (direction_true(grid, x, y, z, "right", side)) {
        vector<Vertex> rightSurface = {
                {voxelMaxX, voxelMinY, voxelMinZ},
                {voxelMaxX, voxelMaxY, voxelMinZ},
                {voxelMaxX, voxelMaxY, voxelMaxZ},
                {voxelMaxX, voxelMinY, voxelMaxZ}
        };
        outputSurfaces.push_back(rightSurface);
    }
    if (direction_true(grid, x, y, z, "left", side)) {
        vector<Vertex> leftSurface = {
                {voxelMinX, voxelMinY, voxelMinZ},
                {voxelMinX, voxelMinY, voxelMaxZ},
                {voxelMinX, voxelMaxY, voxelMaxZ},
                {voxelMinX, voxelMaxY, voxelMinZ}
        };
        outputSurfaces.push_back(leftSurface);
    }
    if (direction_true(grid, x, y, z, "up", side)) {
        vector<Vertex> upSurface = {
                {voxelMinX, voxelMinY, voxelMaxZ},
                {voxelMaxX, voxelMinY, voxelMaxZ},
                {voxelMaxX, voxelMaxY, voxelMaxZ},
                {voxelMinX, voxelMaxY, voxelMaxZ}
        };
        outputSurfaces.push_back(upSurface);
    }
    if (direction_true(grid, x, y, z, "down", side)) {
        vector<Vertex> downSurface = {
                {voxelMinX, voxelMinY, voxelMinZ},
                {voxelMinX, voxelMaxY, voxelMinZ},
                {voxelMaxX, voxelMaxY, voxelMinZ},
                {voxelMaxX, voxelMinY, voxelMinZ}
        };
        outputSurfaces.push_back(downSurface);
    }

    return outputSurfaces;
}


vector<double> calculatePolygonDimensions(const vector<vector<Vertex>> &polygons) {
    // The width and length of the room is calculated
    float minX = polygons[0][0].x;
    float maxX = polygons[0][0].x;
    float minY = polygons[0][0].y;
    float maxY = polygons[0][0].y;
    float minZ = polygons[0][0].z;
    float maxZ = polygons[0][0].z;

    for (const auto &polygon: polygons) {
        for (const auto &vertex: polygon) {
            minX = min(minX, vertex.x);
            maxX = max(maxX, vertex.x);
            minY = min(minY, vertex.y);
            maxY = max(maxY, vertex.y);
            minZ = min(minZ, vertex.z);
            maxZ = max(maxZ, vertex.z);
        }
    }

    double width = maxX - minX;
    double length = maxY - minY;
    double height = maxZ - minZ;
    vector<double> values;
    values.push_back(width);
    values.push_back(length);
    values.push_back(height);
    return values;
}

void printModelInfo(const ObjModel &model, bool printGroupDetails) {
    // Print the ObjModel Information
    // Boolean to enable/disable printing of group names

    cout << "Number of vertices: " << model.vertices.size() << endl;
    cout << "Number of normals: " << model.normals.size() << endl;
    cout << "Number of groups: " << model.groups.size() << endl;

    int totalFaces = 0;
    for (const auto &group: model.groups) {
        if (printGroupDetails) {
            cout << "Group: " << group.groupname << ", Faces: " << group.groupFaces.size();
            cout << ", Smoothing: " << group.smoothingParameter << ", Material: " << group.usemtl << endl;
        }
        totalFaces += group.groupFaces.size();
    }

    cout << "Total number of faces: " << totalFaces << endl;
    cout << "-----------------------" << endl;
}

void assignMinMax(ObjModel &model) {
    // Assigns min, max values for all three directions to ObjModel
    // Iterate over all the vertices of the ObjModel and update if needed
    for (const Group &group: model.groups) {
        for (const Face &face: group.groupFaces) {
            for (const int &vertex: face.vertexIndices) {
                if (model.vertices[vertex].x < model.min_x) model.min_x = model.vertices[vertex].x;
                if (model.vertices[vertex].x > model.max_x) model.max_x = model.vertices[vertex].x;
                if (model.vertices[vertex].y < model.min_y) model.min_y = model.vertices[vertex].y;
                if (model.vertices[vertex].y > model.max_y) model.max_y = model.vertices[vertex].y;
                if (model.vertices[vertex].z < model.min_z) model.min_z = model.vertices[vertex].z;
                if (model.vertices[vertex].z > model.max_z) model.max_z = model.vertices[vertex].z;
            }
        }
    }
}

pair<vector<Triangle3>, vector<Triangle3>> extractTriangles(const ObjModel &model) {
    // Construct triangles in voxel and model coordinates
    vector<Triangle3> trianglesModelCoordinates;
    vector<Triangle3> trianglesGridCoordinates;

    for (const auto &group: model.groups) {
        for (const auto &face: group.groupFaces) {
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

    return make_pair(trianglesModelCoordinates, trianglesGridCoordinates);
}

void markGrid(const vector<Triangle3> &trianglesModel, const vector<Triangle3> &trianglesGrid,
              const ObjModel &model, VoxelGrid &voxelGrid) {
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

void markSpace(VoxelGrid &voxelGrid, vector<int> start, int label, int nrows_x, int nrows_y, int nrows_z) {
    // Initialize queue with voxels to check
    queue<vector<int>> to_check;
    to_check.push(start);

    // Neighbors of starting voxel
    const vector<vector<int>> neighbours = {{1,  0,  0},
                                            {-1, 0,  0},
                                            {0,  1,  0},
                                            {0,  -1, 0},
                                            {0,  0,  1},
                                            {0,  0,  -1}};

    // Until the queue is empty
    while (!to_check.empty()) {
        // Check current voxel which is on top of queue
        vector<int> current_vox = to_check.front();
        to_check.pop();
        int x = current_vox[0], y = current_vox[1], z = current_vox[2];
        // If the voxel is not marked yet, label it
        if (voxelGrid(x, y, z) == 0) {
            voxelGrid(x, y, z) = label;

            // Construct each neighbour
            for (const vector<int> &neighbour: neighbours) {
                int nx = x + neighbour[0];
                int ny = y + neighbour[1];
                int nz = z + neighbour[2];

                // Make sure to work within bounds
                if (nx >= 0 && nx < nrows_x && ny >= 0 && ny < nrows_y && nz >= 0 && nz < nrows_z &&
                    voxelGrid(nx, ny, nz) == 0) {
                    // If the neighbour is within bounds and still unmarked, push to to_check
                    to_check.push({nx, ny, nz});
                }
            }
        }

    }


}


#endif //GEO1004_ASSIGNMENT3_UTILITIES_H
