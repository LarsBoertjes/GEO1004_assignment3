#ifndef GEO1004_ASSIGNMENT3_UTILITIES_H
#define GEO1004_ASSIGNMENT3_UTILITIES_H

#include "ObjModel.h"
#include <string>

using namespace std;

ObjModel readObjFile(const std::string& filePath);
void printModelInfo(const ObjModel& model, bool printGroupDetails);

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

#endif //GEO1004_ASSIGNMENT3_UTILITIES_H


