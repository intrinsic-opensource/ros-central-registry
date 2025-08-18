def generate_ros_version_defines():
    package_name = native.module_name()
    package_version = native.module_version()
    package_version_parts = package_version.split(".")
    package_version_major = package_version_parts[0] if len(package_version_parts) > 0 else ""
    package_version_minor = package_version_parts[1] if len(package_version_parts) > 1 else ""
    package_version_patch = package_version_parts[2] if len(package_version_parts) > 2 else ""
    return [
        "PROJECT_NAME_UPPER={}".format(package_name.upper()),
        "VERSION_MAJOR={}".format(package_version_major),
        "VERSION_MINOR={}".format(package_version_minor),
        "VERSION_PATCH={}".format(package_version_patch),
        "VERSION_STR={}".format(package_version),
    ]
