load("@rules_cc//cc:defs.bzl", "cc_test")

cc_test(
    name = "visibility_polygon_test",
    srcs = [
        "visibility_polygon_test.cc",
    ],
    deps = [
        "//lib:utility",
        "//lib:visibility_polygon",
        "@gtest",
        "@gtest//:gtest_main",  # Only if the test *.cc has no main()
    ],
)
