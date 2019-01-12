/**
 * python_bindings.cc
 *
 * Copyright 2019. All Rights Reserved.
 * IPRL
 *
 * Created: January 10, 2019
 *
 * Authors: Toki Migimatsu
 */

#include <Eigen/Eigen>

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>

#include "articulated_body.h"

namespace FrankaPanda {

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_MODULE(spatialdyn_frankapanda, m) {

  // Articulated body
  py::class_<ArticulatedBody, SpatialDyn::ArticulatedBody>(m, "ArticulatedBody")
      .def(py::init<>())
      .def(py::init<const std::string&>())
      .def(py::init<const SpatialDyn::ArticulatedBody&>())
      .def_property("q",
                    (const Eigen::VectorXd& (ArticulatedBody::*)(void) const) &ArticulatedBody::q,
                    &ArticulatedBody::set_q)
      .def_property("dq",
                    (const Eigen::VectorXd& (ArticulatedBody::*)(void) const) &ArticulatedBody::dq,
                    &ArticulatedBody::set_dq)
      .def_property("inertia_compensation", &ArticulatedBody::inertia_compensation,
                    &ArticulatedBody::set_inertia_compensation)
      .def_property("stiction_coefficients", &ArticulatedBody::stiction_coefficients,
                    &ArticulatedBody::set_stiction_coefficients)
      .def_property("stiction_activations", &ArticulatedBody::stiction_activations,
                    &ArticulatedBody::set_stiction_activations)
      .def_property_readonly("franka_panda", &ArticulatedBody::franka_panda)
      .def("add_load", &ArticulatedBody::AddLoad, "inertia"_a, "idx_link"_a = -1)
      .def("replace_load", &ArticulatedBody::ReplaceLoad, "inertia"_a, "idx_link"_a = -1)
      .def("clear_load", &ArticulatedBody::ClearLoad, "idx_link"_a = -1);

  m.def("friction", py::overload_cast<const ArticulatedBody&, Eigen::Ref<const Eigen::VectorXd>>(&Friction), "ab"_a, "tau"_a);

}

}  // namespace SpatialDyn
