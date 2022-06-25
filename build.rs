extern crate pkg_config;

fn main() {
    println!("cargo:rerun-if-changed=src/main.rs");
    println!("cargo:rerun-if-changed=src/cpp/multibody/model.cpp");
    println!("cargo:rerun-if-changed=src/cpp/include/pinocchio-rust/multibody/model.hpp");

    let pinocchio= pkg_config::Config::new().atleast_version("2.6.4").statik(false).probe("pinocchio").unwrap();
    println!("{:?}", pinocchio);
    println!("cargo:rustc-link-lib=dylib=stdc++");
    for lib in pinocchio.libs {
        println!("cargo:rustc-link-lib=dylib={}", lib);
    } 
    for path in pinocchio.link_paths {
        println!("cargo:rustc-link-search=all={}", path.as_os_str().to_str().unwrap());
    } 
    // for include in pinocchio.include_paths {
    //     println!("cargo:rustc-flags=-I {}", include.as_os_str().to_str().unwrap());
    // }
    cxx_build::bridge("src/multibody/model.rs")
        .file("src/cpp/src/multibody/model.cpp")
        .include("/usr/include/eigen3")
        .include("/opt/openrobots/include")
        .include("src/cpp/include")
        .flag_if_supported("-std=c++14")
        .flag_if_supported("-O3")
        .flag_if_supported("-NDEBUG")
        .flag_if_supported("-march=native")
        .compile("model");
    // cxx_build::bridge("src/multibody/data.rs")
    //     .file("src/cpp/src/multibody/data.cpp")
    //     .include("/usr/include/eigen3")
    //     .include("/opt/openrobots/include")
    //     .include("src/cpp/include")
    //     // .flag_if_supported("-DPINOCCHIO_WITH_URDFDOM")
    //     .compile("data");


}