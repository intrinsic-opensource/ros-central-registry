def multi_test(names = [], **kwargs):
    tests = []
    for name in names:
        native.cc_test(
            name = name.replace("/", "_") + '_test',
            srcs = ["tests/" + name + ".cxx"],
            **kwargs
        )
        tests.append(":" + name)

    return tests