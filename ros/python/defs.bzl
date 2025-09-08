load("@rules_python//python:defs.bzl", _py_test = "py_test")

def py_pytest(name, srcs, deps=[], args=[], **kwargs):
    _py_test(
        name = name,
        main = "@ros//python:pytest_main.py",
        srcs = srcs + ["@ros//python:pytest_main.py"],
        deps = deps + ["@ros//python:pytest"],
        args = args + ["$(location :%s)" % s for s in srcs],
        **kwargs,
    )