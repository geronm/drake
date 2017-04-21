# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:cpplint.bzl", "cpplint")
load(
    "//tools:drake.bzl",
    "drake_cc_binary",
)

package(default_visibility = ["//visibility:public"])

drake_cc_binary(
    name = "sliding_bricks",
    srcs = ["sliding_bricks.cc"],
    add_test_rule = 1,
    data = ["sliding_brick.urdf"],
    test_rule_args = [
        " --sim_duration=0.01",
        " --playback=false",
    ],
    deps = [
        "//drake/common:drake_path",
        "//drake/lcm",
        "//drake/multibody:rigid_body_tree_construction",
        "//drake/multibody/parsers",
        "//drake/multibody/rigid_body_plant",
        "//drake/multibody/rigid_body_plant:drake_visualizer",
        "//drake/systems/analysis",
        "//drake/systems/primitives:constant_vector_source",
        "@gflags//:gflags",
    ],
)

drake_cc_binary(
    name = "rigid_gripper",
    srcs = ["rigid_gripper.cc"],
    add_test_rule = 1,
    data = [
        "rigid_gripper.urdf",
        "//drake/multibody:models",
    ],
    test_rule_args = [
        " --sim_duration=0.01",
        " --playback=false",
    ],
    deps = [
        "//drake/common:drake_path",
        "//drake/lcm",
        "//drake/multibody:rigid_body_tree_construction",
        "//drake/multibody/parsers",
        "//drake/multibody/rigid_body_plant",
        "//drake/multibody/rigid_body_plant:contact_results_to_lcm",
        "//drake/multibody/rigid_body_plant:drake_visualizer",
        "//drake/systems/analysis",
        "@gflags//:gflags",
    ],
)


drake_cc_binary(
    name = "soft_hand",
    srcs = ["soft_hand.cc"],
    add_test_rule = 1,
    data = [
        "soft_hand_knut_twofingers.urdf",
        "soft_hand_box_twofingers.urdf",
        "//drake/multibody:models",
    ],
    test_rule_args = [
        " --sim_duration=0.01",
        " --playback=false",
    ],
    deps = [
        "//drake/common:drake_path",
        "//drake/lcm",
        "//drake/common",
        "//drake/multibody:rigid_body_tree_construction",
        "//drake/multibody/parsers",
        "//drake/multibody/rigid_body_plant",
        "//drake/multibody/rigid_body_plant:contact_results_to_lcm",
        "//drake/multibody/rigid_body_plant:drake_visualizer",
        "//drake/systems/primitives:constant_vector_source",
        "//drake/systems/primitives:gain",
        "//drake/systems/primitives:multiplexer",
        "//drake/systems/primitives:demultiplexer",
        "//drake/systems/analysis",
        "//drake/lcmtypes:viewer2",
        "@gflags//:gflags",
    ],
)


drake_cc_binary(
    name = "soft_hand_jacobian_control",
    srcs = ["soft_hand_jacobian_control.cc"],
    add_test_rule = 1,
    data = [
        "soft_hand_knut_twofingers.urdf",
        "soft_hand_box_twofingers.urdf",
        "soft_hand_box_twofingers_jc.urdf",
        "//drake/multibody:models",
    ],
    test_rule_args = [
        " --sim_duration=0.01",
        " --playback=false",
    ],
    deps = [
        "//drake/common:drake_path",
        "//drake/lcm",
        "//drake/common",
        "//drake/multibody:rigid_body_tree_construction",
        "//drake/multibody/parsers",
        "//drake/multibody/rigid_body_plant",
        "//drake/multibody/rigid_body_plant:contact_results_to_lcm",
        "//drake/multibody/rigid_body_plant:drake_visualizer",
        "//drake/systems/primitives:constant_vector_source",
        "//drake/systems/primitives:gain",
        "//drake/systems/primitives:multiplexer",
        "//drake/systems/primitives:demultiplexer",
        "//drake/systems/controllers:pid_controller",
        "//drake/systems/analysis",
        "//drake/lcmtypes:viewer2",
        "//drake/examples/contact_model/qp_control:qp_controller",
        "//drake/examples/contact_model/qp_control:qp_controller_system",
        "@gflags//:gflags",
    ],
)


drake_cc_binary(
    name = "soft_hand_knut_screw",
    srcs = ["soft_hand_knut_screw.cc"],
    add_test_rule = 1,
    data = [
        "soft_hand_knut_twofingers_jc.urdf",
        "//drake/multibody:models",
    ],
    test_rule_args = [
        " --sim_duration=0.01",
        " --playback=false",
    ],
    deps = [
        "//drake/common:drake_path",
        "//drake/lcm",
        "//drake/common",
        "//drake/multibody:rigid_body_tree_construction",
        "//drake/multibody/parsers",
        "//drake/multibody/rigid_body_plant",
        "//drake/multibody/rigid_body_plant:contact_results_to_lcm",
        "//drake/multibody/rigid_body_plant:drake_visualizer",
        "//drake/systems/primitives:constant_vector_source",
        "//drake/systems/primitives:gain",
        "//drake/systems/primitives:multiplexer",
        "//drake/systems/primitives:demultiplexer",
        "//drake/systems/controllers:pid_controller",
        "//drake/systems/analysis",
        "//drake/lcmtypes:viewer2",
        "//drake/examples/contact_model/qp_control:qp_controller",
        "//drake/examples/contact_model/qp_control:qp_controller_system",
        "@gflags//:gflags",
    ],
)

drake_cc_binary(
    name = "bowling_ball",
    srcs = ["bowling_ball.cc"],
    add_test_rule = 1,
    data = [
        ":bowling_models",
    ],
    test_rule_args = [
        " --sim_duration=0.01",
        " --playback=false",
    ],
    deps = [
        "//drake/common:drake_path",
        "//drake/lcm",
        "//drake/multibody:rigid_body_tree_construction",
        "//drake/multibody/parsers",
        "//drake/multibody/rigid_body_plant",
        "//drake/multibody/rigid_body_plant:drake_visualizer",
        "//drake/systems/analysis",
        "//drake/systems/lcm",
        "@gflags//:gflags",
    ],
)

filegroup(
    name = "bowling_models",
    srcs = [
        "bowling_ball.urdf",
        "pin.obj",
        "pin.urdf",
    ],
)

cpplint()
