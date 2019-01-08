/**
 * spatial_dyn.cc
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: September 7, 2018
 * Authors: Toki Migimatsu
 */

#include <Eigen/Eigen>

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>

#include "FrankaPanda/model.h"

namespace FrankaPanda {

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_MODULE(frankapanda, m) {

  // Articulated body
  py::class_<Model>(m, "Model")
      .def(py::init<>())
      .def_property_readonly("dof", &Model::dof)
      .def_property("q", &Model::q, &Model::set_q)
      .def_property("dq", &Model::dq, &Model::set_dq)
      .def_property("m_load", &Model::m_load, &Model::set_m_load)
      .def_property("com_load", &Model::com_load, &Model::set_com_load)
      .def_property("I_com_load", &Model::I_com_load, &Model::set_I_com_load)
      .def_property_readonly("I_com_load_matrix", &Model::I_com_load_matrix)
      .def_property_readonly("g", &Model::g);

  // Forward kinematics
  m.def("cartesian_pose",
        [](const Model& model, int link) -> Eigen::Matrix4d {
          Eigen::Isometry3d T = CartesianPose(model, link);
          return T.matrix();
        }, "model"_a, "link"_a = -1)
   .def("jacobian", &Jacobian, "model"_a, "link"_a = -1)
   .def("inertia", &Inertia, "model"_a)
   .def("centrifugal_coriolis", &CentrifugalCoriolis, "model"_a)
   .def("gravity", &Gravity, "model"_a);

}

}  // namespace SpatialDyn
