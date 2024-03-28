#ifndef GEO1004_ASSIGNMENT3_OBJMODEL_H
#define GEO1004_ASSIGNMENT3_OBJMODEL_H

#include <vector>
#include <string>

struct Vertex {
    float x, y, z;
};

struct Normal {
    float nx, ny, nz;
};

struct Face {
    std::vector<int> vertexIndices;
    std::vector<int> normalIndices;
};

struct Group {
    std::vector<Face> groupFaces;
    int smoothingParameter = 0;
    std::string usemtl;
    std::string groupname;
};

struct ObjModel {
    std::vector<Group> groups;
    std::vector<Vertex> vertices;
    std::vector<Normal> normals;
};

#endif //GEO1004_ASSIGNMENT3_OBJMODEL_H
