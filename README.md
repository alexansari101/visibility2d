# visibility2d

Library to compute a visiblity polygon based on an observer's location and and the locations of polygonal obstacles.

## Build and Run

This package has been tested on **Ubuntu 20.04**.

To remove build toolchain dependencies, bazel commands use `--config=llvm_ext` to build with an external llvm toolchain (specified in `.bazelrc` and `WORKSPACE`).

_Notes:_

- _The first time a user issues a bazel build command, the build process will be slower as the system needs to download the external toolchain._
- `bazel clean --expunge` _will clean and remove any downladed external dependences, e.g., the llvm toolchain._

### Build Tests:

`bazel test --config=llvm_ext //test:all`
