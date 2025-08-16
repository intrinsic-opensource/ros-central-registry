use std::path::{Path, PathBuf};
use std::{env, io, fs};

mod buildrs;

pub fn get_build_rs_path() -> PathBuf {
    let path = env::current_dir().unwrap_or_default();
    path.to_path_buf()
}

pub fn get_out_path() -> PathBuf {
    let out_dir_env = env::var("OUT_DIR").unwrap_or_default();
    let path = Path::new(&out_dir_env);
    println!("Writing to {} ", path.display());
    path.to_path_buf()
}

fn copy_dir_all(src: impl AsRef<Path>, dst: impl AsRef<Path>) -> io::Result<()> {
    fs::create_dir_all(&dst)?;
    for entry in fs::read_dir(src)? {
        let entry = entry?;
        let ty = entry.file_type()?;
        if ty.is_dir() {
            copy_dir_all(entry.path(), dst.as_ref().join(entry.file_name()))?;
        } else {
            fs::copy(entry.path(), dst.as_ref().join(entry.file_name()))?;
        }
    }
    Ok(())
}

fn main() {
    buildrs::opaque_types_generator::generate_opaque_types();
    buildrs::cbindgen_generator::generate_c_headers();

    // Note that this generates in the source tree. We need to copy the products
    // to the OUTPUT_DIR for use by the rest of the Bazel build!
    let _ = copy_dir_all(get_build_rs_path().join("src/opaque_types"), get_out_path().join("src/opaque_types"));
    //let _ = copy_dir_all(get_build_rs_path().join("src"), get_out_path().join("src"));

    // Signals to cargo to trigger rebuilds.
    println!("cargo:rerun-if-changed=build.rs");
    println!("cargo:rerun-if-changed=buildrs");
    println!("cargo:rerun-if-changed=src");
    println!("cargo:rerun-if-changed=splitguide.yaml");
    println!("cargo:rerun-if-changed=cbindgen.toml");
    println!("cargo:rerun-if-changed=build-resources");
    println!("cargo:rerun-if-changed=include");
    if std::env::var("CARGO_CFG_TARGET_OS").as_deref() == Ok("linux") {
        let name = std::env::var("CARGO_PKG_NAME").unwrap();
        // Create the shared library name by removing hyphens from the pkg_name
        let soname = format!("lib{}.so", name.replace('-', ""));
        println!("cargo:rustc-cdylib-link-arg=-Wl,-soname,{}", soname);
    }
}
