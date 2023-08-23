#ifndef OCTREE_PY_HPP
#define OCTREE_PY_HPP
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <vector>
#include "../Octree/octree.h"
#include "../Octree/octree_container.h"

namespace py = pybind11;
using namespace OrthoTree;

class OctreePy
{
private:
    float euclidean_distance(double x1, double y1, double z1, double x2, double y2, double z2)
    {
        return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2) + pow(z1 - z2, 2));
    }

public:
    OctreePy(int depth)
    {
        depth_ = depth;
        array<array<double, 3>, 0> constexpr points;
        c_tree_ = OctreePointC(points, depth);
    }

    void create(vector<array<double, 3>> points, int depth)
    {
        depth_ = depth;
        c_tree_ = c_tree_.Create(points, depth);
    }

    int get_depth()
    {
        return depth_;
    }

    int get_size()
    {
        return c_tree_.GetData().size();
    }

    void addPoint(array<double, 3> point)
    {
        std::cout << "Adding point " << point[0] << " " << point[1] << " " << point[2] << std::endl;
        c_tree_.Add(point);
    }

    vector<long unsigned int> rangeSearch(array<array<double, 3>, 2> search_box)
    {
        return c_tree_.RangeSearch(BoundingBox3D{search_box[0], search_box[1]});
    }

    vector<long unsigned int> searchRadius(array<double, 3> position, double radius)
    {
        auto const search_box_cpp = BoundingBox3D{{position[0] - radius, position[1] - radius, position[2] - radius}, {position[0] + radius, position[1] + radius, position[2] + radius}};
        vector<long unsigned int> ids_in_box = c_tree_.RangeSearch(search_box_cpp);
        for (int i = 0; i < ids_in_box.size(); i++)
        {
            int id = ids_in_box[i];
            if (euclidean_distance(position[0], position[1], position[2], c_tree_.GetData()[id][0], c_tree_.GetData()[id][1], c_tree_.GetData()[id][2]) > radius)
                ids_in_box.erase(ids_in_box.begin() + i);
        }
        return ids_in_box;
    }

    long unsigned int getNearestNeighbor(array<double, 3> point)
    {
        return c_tree_.GetNearestNeighbors(point, 1)[0];
    }

    vector<long unsigned int> getNearestNeighbors(array<double, 3> point, int k)
    {
        return c_tree_.GetNearestNeighbors(point, k);
    }

private:
    OctreePointC c_tree_; // todo we should use the not containered version instead so that we don't copy the location data
    int depth_;
};

PYBIND11_MODULE(octree_py, m)
{
    m.doc() = "Octree Python bindings"; // optional module docstring

    py::class_<OctreePy>(m, "Octree")
        .def(py::init<int>())
        .def("create", &OctreePy::create)
        .def("get_depth", &OctreePy::get_depth)
        .def("get_size", &OctreePy::get_size)
        .def("add_point", &OctreePy::addPoint)
        .def("range_search", &OctreePy::rangeSearch)
        .def("search_radius", &OctreePy::searchRadius)
        .def("get_nearest_neighbor", &OctreePy::getNearestNeighbor)
        .def("get_nearest_neighbors", &OctreePy::getNearestNeighbors);
}
#endif // OCTREE_PY_HPP