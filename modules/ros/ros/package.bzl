load(":interface.bzl", "RosInterfaceInfo")

RosPackageInfo = provider(
    "Provides info for interface code generation.",
    fields = {
        "src": "The source file defining an interface.",
        "deps": "The interfaces that this interface depends on.",
    }
)

def _ros_package_impl(ctx):
    return [
        DefaultInfo(files = depset(ctx.files.src)),
        RosPackageInfo(
            src = ctx.files.src,
            deps = depset(
                direct = [
                    dep[RosPackageInfo].src for dep in ctx.attr.deps
                ],
                transitive = [
                    dep[RosPackageInfo].deps for dep in ctx.attr.deps
                ],
            ),
        ),
    ]

ros_package = rule(
    implementation = _ros_package_impl,
    attrs = {
        "interfaces": attr.label_list(providers = [RosInterfaceInfo]),
        "libraries": attr.label_list(),
        "binaries": attr.label_list(),
        "data": attr.label_list(),
        "tests": attr.label_list(),
    },
    provides = [
        RosPackageInfo
    ],
)
