import os
import sys
from sphinx.cmd.build import main

if __name__ == "__main__":
    # Ensure subprocesses can find dependencies
    os.environ["PYTHONPATH"] = os.pathsep.join(sys.path)
    # The arguments are passed to the sphinx.cmd.build.main function.
    sys.exit(main(sys.argv[1:]))
