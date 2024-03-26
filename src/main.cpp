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

struct ObjModel {
    vector<Vertex> vertices;
    vector<Normal> normals;
    vector<Face> faces;
};

ObjModel readObjFile(const string& filePath);

int main(int argc, const char * argv[]) {

    // Step 1: read the obj file content into memory
    ObjModel model = readObjFile("../data/IfcOpenHouse_IFC2x3.obj");

    cout << "Loaded " << model.vertices.size() << " vertices, "
         << model.normals.size() << " normals, and "
         << model.faces.size() << " faces." << endl;


    return 0;
}

ObjModel readObjFile(const string& filePath) {
    ObjModel model;
    string line;
    ifstream objFile(filePath);

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
                    int vIndex = stoi(vertex.substr(0, pos1)) - 1; // OBJ indices are 1-based
                    int nIndex = stoi(vertex.substr(pos1 + 2)) - 1;
                    f.vertexIndices.push_back(vIndex);
                    f.normalIndices.push_back(nIndex);
                }
                model.faces.push_back(f);
            }
        }
        objFile.close();
    } else {
        cout << "Unable to open file" << endl;
    }

    return model;
}

