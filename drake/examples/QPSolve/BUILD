# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:cpplint.bzl", "cpplint")
# load("//tools:drake.bzl", "drake_cc_googletest", "drake_cc_library")
load("//tools:drake.bzl", "drake_cc_binary", "drake_cc_library")

package(
    default_visibility = [":__subpackages__"],
)

drake_cc_library(
    name = "qp_solve_hdr",
    srcs = [],
    hdrs = ["qp_solve.h"],
    deps = [
        "//drake/common",
        "//drake/math:geometric_transform",
    ],
)

drake_cc_binary(
    name = "qp_solve",
    srcs = ["qp_solve.cc"],
    deps = [
        ":qp_solve_hdr",
        "//drake/common",
        "//drake/math:gradient",
        "//drake/solvers:mathematical_program",
    ],
)


# drake_cc_binary(
#     name = "qp_solve",
#     srcs = ["qp_solve.cc"],
#     hdrs = ["qp_solve.h"],
#     deps = [
#         "//drake/common",
#         "//drake/math:gradient",
#         "//drake/solvers:mathematical_program",
#     ],
# )

cpplint()
