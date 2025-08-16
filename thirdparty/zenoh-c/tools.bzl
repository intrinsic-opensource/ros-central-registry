def multi_test(names = [], **kwargs):
    tests = []
    for name in names:
        native.cc_test(
            name = 'test_' + name.replace("/", "_"),
            srcs = [       
                "tests/z_int_helpers.h",
                "tests/" + name + ".c"
            ],
            **kwargs
        )
        tests.append(":" + name)

    return tests


def _cargo_build_stdout_capture_impl(ctx):
    output_file = ctx.outputs.out
    manifest_file = ctx.file.manifest
    inputs = ctx.files.srcs
    cargo_features = ctx.attr.cargo_features

    # Convert a list of features to a string of -F ... arguments
    cargo_feature_str = ""
    for cargo_feature in cargo_features:
        cargo_feature_str += "-F %s " % cargo_feature

    # Construct a command to cargo build and pipe the output to a file. Note that we are adding
    # the suffix || true to override the failing return code with a pass to avoid aborting. 
    command_str = "cargo build --no-default-features -F panic %s --manifest-path %s &> %s || true" % (
        cargo_feature_str, manifest_file.path, output_file.path)
    # print("Building with: %s" % command_str)

    # Run the command.
    ctx.actions.run_shell(
        inputs = inputs,
        outputs = [
            output_file
        ],
        command = command_str,
        mnemonic = "CargoBuildStdoutCapture",
        progress_message = "Building opaque_types and capturing stdout to %s" % output_file.short_path,
        use_default_shell_env = True,
    )

    return [DefaultInfo(files = depset([output_file]))]

cargo_build_stdout_capture = rule(
    implementation = _cargo_build_stdout_capture_impl,
    attrs = {
        "manifest": attr.label(
            allow_single_file = True,
            mandatory = True,
            doc = "The path to the Cargo.toml manifest in the project.",
        ),
        "srcs": attr.label_list(
            allow_files=True,
            mandatory=True,
            doc="All rust source files for the project."
        ),
        "cargo_features": attr.string_list(
            doc = "A list of features for the cargo build.",
            default = [],
        ),
        "out": attr.output(
            mandatory=True,
            doc="The output file to which `cargo build` stdout will be written."
        ),
    },
    doc = "Builds a Rust project using `cargo build` and captures its stdout to a specified file."
)
