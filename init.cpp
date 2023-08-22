#ifndef OCTREE_PY_HPP
#define OCTREE_PY_HPP
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "./Octree/octree.h"
#include "./Octree/octree_container.h"

namespace py = pybind11;
using namespace OrthoTree;

PYBIND11_MODULE(octree, m) {
    m.doc() = "Octree Python bindings"; // optional module docstring

    py::class_<OctreePoint>(m, "Octree")
        .def(py::init<>());
        //.def("setName", &Pet::setName)
}
#endif // OCTREE_PY_HPP