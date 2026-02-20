import os
import shutil
import sys
import runpy

def main():
    # Detect workspace directory from Bazel environment
    workspace_dir = os.environ.get("BUILD_WORKSPACE_DIRECTORY")
    if not workspace_dir:
        print("Error: BUILD_WORKSPACE_DIRECTORY not set. Please run with 'bazel run //docs:preview'.", file=sys.stderr)
        sys.exit(1)

    docs_dir = os.path.join(workspace_dir, "docs", "source")

    # Use a build directory inside bazel-bin to avoid cluttering source. Delete it
    # at startup to avoi caching a navigation structure across runs.
    build_dir = os.path.join(workspace_dir, "bazel-bin", "docs", "html_preview")
    if os.path.exists(build_dir):
        shutil.rmtree(build_dir)
    os.makedirs(build_dir, exist_ok=True)

    print(f"Starting sphinx-autobuild...")
    print(f"  Source: {docs_dir}")
    print(f"  Build:  {build_dir}")

    # Ensure subprocesses spawned by sphinx-autobuild can find dependencies
    # explicitly propagating the current sys.path to PYTHONPATH.
    os.environ["PYTHONPATH"] = os.pathsep.join(sys.path)

    try:
        import sphinx
        print(f"  Sphinx: {os.path.dirname(sphinx.__file__)}")
    except ImportError:
        print("Error: Could not import sphinx module directly.", file=sys.stderr)

    # Set up arguments for sphinx-autobuild
    sys.argv = [
        "sphinx-autobuild",
        docs_dir,
        build_dir,
        "--port", "8000",
        "--open-browser",
        "--watch", docs_dir,
        "--re-ignore", r".*\.swp$",
        "--re-ignore", r".*\.swo$",
        "--re-ignore", r"__pycache__",
        "--re-ignore", r".*\.pyc$",
    ]
    
    # Run sphinx-autobuild in the same process to preserve environment
    try:
        runpy.run_module("sphinx_autobuild", run_name="__main__")
    except KeyboardInterrupt:
        print("\nStopping documentation server.")
    except SystemExit as e:
        sys.exit(e.code)
    except Exception as e:
        print(f"Error running sphinx-autobuild: {e}", file=sys.stderr)
        sys.exit(1)

if __name__ == "__main__":
    main()
