/**
 * franka_panda.cc
 *
 * Copyright 2019. All Rights Reserved.
 * IPRL
 *
 * Created: January 07, 2019
 * Authors: Toki Migimatsu
 */

#include <Eigen/Eigen>

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>

#include "franka_panda/model.h"

namespace franka_panda {

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
      .def_property("I_com_load_matrix", &Model::I_com_load_matrix, &Model::set_I_com_load_matrix)
      .def("set_load", &Model::set_load)
      .def_property_readonly("g", &Model::g)
      .def_property("inertia_compensation", &Model::inertia_compensation, &Model::set_inertia_compensation)
      .def_property("stiction_coefficients", &Model::stiction_coefficients, &Model::set_stiction_coefficients)
      .def_property("stiction_activations", &Model::stiction_activations, &Model::set_stiction_activations);

  // Dynamics
  m.def("cartesian_pose",
        [](const Model& model, int link) -> Eigen::Matrix4d {
          Eigen::Isometry3d T = CartesianPose(model, link);
          return T.matrix();
        }, "model"_a, "link"_a = -1)
   .def("jacobian", &Jacobian, "model"_a, "link"_a = -1)
   .def("inertia", &Inertia, "model"_a)
   .def("centrifugal_coriolis", &CentrifugalCoriolis, "model"_a)
   .def("gravity", &Gravity, "model"_a)
   .def("friction", &Friction, "model"_a, "tau"_a);

}

}  // namespace franka_panda
