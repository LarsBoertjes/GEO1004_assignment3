#include <iostream>
#include <fstream>
#include <vector>
#include <sstream>
#include <string>
#include "json.hpp"

// CGAL libraries
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>

using json = nlohmann::json;
using namespace std;

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef K::Point_3 Point3;

// Structures to store vertex and normal data
struct Vertex {
    float x, y, z;
};

struct Normal {
    float nx, ny, nz;
};

struct Face {
    vector<int> vertexIndices;
    vector<int> normalIndices;

};

struct Group {
    vector<Face> groupFaces;
    int smoothingParameter = 0;
    string usemtl;
    string groupname;
};

struct ObjModel {
    vector<Group> groups;
    vector<Vertex> vertices;
    vector<Normal> normals;
};

ObjModel readObjFile(const string& filePath);
void printModelInfo(const ObjModel& model, bool printGroupDetails);

int main(int argc, const char * argv[]) {

    // Step 1: read the obj file content into memory
    ObjModel model = readObjFile("../data/IfcOpenHouse_IFC4.obj");


    // Think about removing the slabs bigger than the building footprint




    return 0;
}

ObjModel readObjFile(const string& filePath) {
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

    cout << "Successfully read OBJ file in memory with following content: " << endl;
    printModelInfo(model, false);

    return model;
}

void printModelInfo(const ObjModel& model, bool printGroupDetails) {
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