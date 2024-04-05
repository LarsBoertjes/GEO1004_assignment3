#ifndef GEO1004_ASSIGNMENT3_VOXELGRID_H
#define GEO1004_ASSIGNMENT3_VOXELGRID_H

#include <vector>
#include <cassert>

struct VoxelGrid {
    // -- This structure can be used in the following way:
    // VoxelGrid my_building_grid(rows_x, rows_y, rows_z)
    // unsigned int v = my_building_grid(x,y,z)
    // my_building_grid(x,y,z) = 1
    std::vector<unsigned int> voxels;
    unsigned int max_x, max_y, max_z;
    int nrows_x, nrows_y, nrows_z;
    float resolution;

    VoxelGrid(unsigned int x, unsigned int y, unsigned int z) {
        max_x = x;
        max_y = y;
        max_z = z;
        nrows_x = x;
        nrows_y = y;
        nrows_z = z;

        unsigned int total_voxels = x*y*z;
        voxels.reserve(total_voxels);
        for (unsigned int i = 0; i < total_voxels; ++i) voxels.push_back(0);
    }

    unsigned int &operator()(const unsigned int &x, const unsigned int &y, const unsigned int &z) {
        assert(x >= 0 && x < max_x);
        assert(y >= 0 && y < max_y);
        assert(z >= 0 && z < max_z);
        return voxels[x + y*max_x + z*max_x*max_y];
    }

    unsigned int operator()(const unsigned int &x, const unsigned int &y, const unsigned int &z) const {
        assert(x >= 0 && x < max_x);
        assert(y >= 0 && y < max_y);
        assert(z >= 0 && z < max_z);
        return voxels[x + y*max_x + z*max_x*max_y];
    }

    Vertex model_to_voxel(const Vertex& vertex, float min_x, float min_y, float min_z) const {
        Vertex voxelVertex;

        float out_x = static_cast<unsigned int>((vertex.x - min_x) / resolution);
        float out_y = static_cast<unsigned int>((vertex.y - min_y) / resolution);
        float out_z = static_cast<unsigned int>((vertex.z - min_z) / resolution);

        voxelVertex.x = out_x;
        voxelVertex.y = out_y;
        voxelVertex.z = out_z;

        return voxelVertex;
    }
};

#endif //GEO1004_ASSIGNMENT3_VOXELGRID_H
