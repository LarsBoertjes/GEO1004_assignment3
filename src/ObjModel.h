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
    std::string name;
};

struct Triangle {
    Vertex vertices[3];
};

struct Group {
    std::vector<Face> groupFaces;
    int smoothingParameter = 0;
    std::string usemtl;
    std::string groupname;

    // Added following for removal of street
    float boundingBox;
    float min_x = std::numeric_limits<float>::max(), max_x = std::numeric_limits<float>::lowest();
    float min_y = std::numeric_limits<float>::max(), max_y = std::numeric_limits<float>::lowest();
    float min_z = std::numeric_limits<float>::max(), max_z = std::numeric_limits<float>::lowest();
};

struct ObjModel {
    std::vector<Group> groups;
    std::vector<Vertex> vertices;
    std::vector<Vertex> model_vertices;
    std::vector<Normal> normals;
    float min_x = std::numeric_limits<float>::max(), max_x = std::numeric_limits<float>::lowest();
    float min_y = std::numeric_limits<float>::max(), max_y = std::numeric_limits<float>::lowest();
    float min_z = std::numeric_limits<float>::max(), max_z = std::numeric_limits<float>::lowest();

    // Function to add new vertex to the model
    int addVertex(float x, float y, float z) {
        Vertex vertex;
        vertex.x = x;
        vertex.y = y;
        vertex.z = z;
        vertices.emplace_back(vertex);
        return vertices.size() - 1;
    }
};


#endif //GEO1004_ASSIGNMENT3_OBJMODEL_H
