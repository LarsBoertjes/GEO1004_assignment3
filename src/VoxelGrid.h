#ifndef GEO1004_ASSIGNMENT3_VOXELGRID_H
#define GEO1004_ASSIGNMENT3_VOXELGRID_H

#include <vector>
#include <cassert>

struct VoxelGrid {
    // -- This structure can be used in the following way:
    // VoxelGrid my_building_grid(rows_x, rows_y, rows_z)
    // unsigned int v = my_building_grid(x,y,z)
    // my_building_grid(x,y,z) = 1

    // -- Add functions like:
    // translate from the model's coordinates to integer coordinates of the voxel grid and vice versa

    std::vector<unsigned int> voxels;
    unsigned int max_x, max_y, max_z;

    VoxelGrid(unsigned int x, unsigned int y, unsigned int z) {
        max_x = x;
        max_y = y;
        max_z = z;
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

    void model_to_voxel(Vertex& vertex, float min_x, float min_y, float min_z) const {
        // Calculate voxel coordinates relative to the minimum coordinates
        float relative_x = vertex.x - min_x;
        float relative_y = vertex.y - min_y;
        float relative_z = vertex.z - min_z;

        // Offset the voxel coordinates to create a boundary of empty voxels
        /*vertex.x_voxel = static_cast<unsigned int>(std::floor(relative_x / voxel_res)) + 1;
        vertex.y_voxel = static_cast<unsigned int>(std::floor(relative_y / voxel_res)) + 1;
        vertex.z_voxel = static_cast<unsigned int>(std::floor(relative_z / voxel_res)) + 1;*/
    }

    void voxel_to_model(Vertex& vertex, float min_x, float min_y, float min_z) const {
        float model_x = vertex.x + min_x;
        float model_y
    }

};

#endif //GEO1004_ASSIGNMENT3_VOXELGRID_H
