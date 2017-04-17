# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:cpplint.bzl", "cpplint")
load("//tools:drake.bzl", "drake_cc_binary", "drake_cc_googletest", "drake_cc_library")


package(
    default_visibility = ["//visibility:public"],
)

# drake_cc_library(
#     name = "qp_controller_common",
#     srcs = ["qp_controller_common.cc"],
#     hdrs = ["qp_controller_common.h"],
#     deps = [
#         "//drake/multibody:rigid_body_tree",
#     ],
# )

drake_cc_library(
    name = "qp_controller",
    srcs = ["qp_controller.cc"],
    hdrs = ["qp_controller.h"],
    deps = [
#        ":qp_controller_common",
        "//drake/common",
        "//drake/math:gradient",
        "//drake/solvers:mathematical_program",
    ],
)

drake_cc_library(
    name = "qp_controller_system",
    srcs = ["qp_controller_system.cc"],
    hdrs = ["qp_controller_system.h"],
    deps = [
        ":qp_controller",
        "//drake/lcmtypes:inverse_dynamics_debug_info",
        "//drake/multibody:rigid_body_tree",
        "//drake/systems/framework:leaf_system",
    ],
)

cpplint()
