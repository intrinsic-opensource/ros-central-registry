# Staged BCR Modules

This directory contains forks of Bazel Central Registry (BCR) modules. They are staged locally to resolve compiler, platform, and toolchain compatibility issues, particularly when targetting **Windows** (via the LLVM Clang-MinGW toolchain), **Linux**, and **macOS**.

Below is a summary of the key fixes applied to these modules:

---

### 1. `llvm` (Clang Compiler Toolchain)
* **Dynamic C++ Runtime on Windows:** PE/COFF does not support dynamic cross-referencing across separate runtime DLLs like ELF does. The build is patched to statically link (absorb) the C++ ABI (`libcxxabi`) and unwinder (`libunwind`) directly into `libc++.dll`, creating a single self-contained runtime DLL.
* **Symbol Exports:** Enables `windows_export_all_symbols` feature for the dynamic runtime, ensuring `libc++.dll` exports the C++ ABI symbols.
* **Shared Library Naming:** Fixes the `cc_shared_library` artifact pattern on Windows to use `lib%{name}.dll` instead of the generic `.so` extension, allowing MinGW's `ld.lld` linker to resolve imports correctly via `-l` flags.

### 2. `rules_cc` (C++ Rules)
* **DLL Runtime Copying:** Windows lacks `rpath` support and loads DLLs from the same directory as the executable (or from `PATH`). The module is patched to copy the C++ runtime library (`libc++.dll`) next to compiled binaries to prevent `STATUS_DLL_NOT_FOUND` runtime crashes.

### 3. `aspect_rules_py` (Python Rules)
* **Windows Support:** Upstream `rules_py` has a POSIX-specific venv launcher strategy. The module is patched to transparently dispatch python target rules (`py_library`, `py_binary`, `py_test`) to standard `rules_python` on Windows.
* **Resource Partitioning:** Moves non-Python source files from `srcs` to `data` to satisfy `rules_python` constraints.

### 4. `mimick` (Mocking Library)
* **Thread-Local Storage:** Clang targeting MinGW on Windows advertises C11 support but does not ship `<threads.h>`. The code is patched to fall back to native Windows TLS (`TlsAlloc`) rather than C11 threads.
* **Platform Packaging:** Outlines custom `config_setting` configurations to map architecture-specific ASM assembly files (trampolines) and PE/ELF/Mach-O PLT implementations.

### 5. `google_benchmark` (Microbenchmarking Library)
* **Compiler Flag Compatibility:** Replaces the MSVC-specific `/std:c++17` flag with the GCC-style `-std=c++17` flag since Windows targets are built using the LLVM toolchain.

### 6. `libjpeg_turbo` (SIMD JPEG Library)
* **Compiler Flag Compatibility:** Replaces MSVC compiler flags (`/Ox`, `-wd4996`) with GCC-style compiler equivalents (`-O3`, `-w`, etc.) for compiling under the Windows Clang-MinGW toolchain.
* **Assembly Compilation:** Automatically configures `nasm` assembly options (`-f elf64` on Linux vs. `-fwin64 -DWIN64 -D__x86_64__` on Windows) to generate correct SIMD assembly targets.

### 7. `openssl` (Cryptography Library)
* **Configuration Mapping:** Translates OpenSSL's complex configurations into clean, platform-specific constant lists of files, sources, and compile definitions (`constants-*.bzl`) for Linux, macOS, and Windows.

### 8. `nasm` (Netwide Assembler)
* **GNU Toolchain Compatibility:** Sets `-DHAVE_CONFIG_H` to override MSVC-specific fallback code paths (which use MSVC-only `123i64` suffixes that Clang rejects).
* **POSIX Fallbacks:** Maps target systems to pre-configured header templates (`config_linux.h`, `config_macos.h`, `config_windows.h`). The Windows configuration explicitly disables POSIX-only APIs (like `mmap` or `getrusage`) to force fallback to portable implementations.

### 9. `asio` / `cyclonedds` / `fastdds` (Networking / DDS)
* **Winsock Conflict Resolution:** Configures include headers and compiler definitions (e.g. `-include winsock2.h`, `-U_WINSOCKAPI_`, and `WIN32_LEAN_AND_MEAN`) to avoid conflicts between `winsock.h` and `winsock2.h` under Clang.
* **Mutex & Real-time Adapters:** Sets `MINGW_COMPILER` layout switches for standard timed mutexes and imports necessary Windows system libraries (`ws2_32`, `iphlpapi`, `bcrypt`, `dbghelp`).
