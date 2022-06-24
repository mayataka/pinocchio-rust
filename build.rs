fn main() {
    cxx_build::bridge("src/multibody/model.rs")
        .file("src/multibody/model.cpp")
        .include("/usr/include/eigen3")
        .include("/opt/openrobots/include")
        .include("src")
        .flag_if_supported("-std=c++14")
        .flag_if_supported("-O3")
        .flag_if_supported("-march=native")
        .flag_if_supported("-NDEBUG")
        .compile("model");
    cxx_build::bridge("src/multibody/data.rs")
        .file("src/multibody/data.cpp")
        .include("/usr/include/eigen3")
        .include("/opt/openrobots/include")
        .include("src")
        .flag_if_supported("-std=c++14")
        .flag_if_supported("-O3")
        .flag_if_supported("-march=native")
        .flag_if_supported("-NDEBUG")
        .compile("data");

    // println!("cargo:rerun-if-changed=src/main.rs");
    // println!("cargo:rerun-if-changed=src/blobstore.cc");
    // println!("cargo:rerun-if-changed=include/blobstore.h");
}