use pkg_config;

fn build_bridge_module(category: &str, module: &str) {
    let rsfile = String::from("src/");
    let rsfile = rsfile + category;
    let rsfile = rsfile + "/";
    let rsfile = rsfile + module;
    let rsfile = rsfile + ".rs";
    let cppfile = String::from("src/cpp/src/");
    let cppfile = cppfile + category;
    let cppfile = cppfile + "/";
    let cppfile = cppfile + module;
    let cppfile = cppfile + ".cpp";
    cxx_build::bridge(&rsfile)
        .file(&cppfile)
        .include("/usr/include/eigen3")
        .include("/opt/openrobots/include")
        .include("src/cpp/include")
        .flag_if_supported("-std=c++17")
        .flag_if_supported("-O3")
        .flag_if_supported("-NDEBUG")
        .flag_if_supported("-march=native")
        .compile(module);
    let rerun = String::from("cargo:rerun-if-changed=");
    let rerun = rerun + &rsfile;
    println!("{}", &rerun);
}

fn main() {
    // Find and link pinocchio-related libraries
    println!("cargo:rustc-link-lib=dylib=stdc++");
    let pinocchio= pkg_config::Config::new().atleast_version("2.6.4").statik(false).probe("pinocchio").unwrap();
    for lib in pinocchio.libs {
        println!("cargo:rustc-link-lib=dylib={}", lib);
    } 
    for path in pinocchio.link_paths {
        println!("cargo:rustc-link-search=all={}", path.as_os_str().to_str().unwrap());
    } 
    build_bridge_module("multibody", "model");
    build_bridge_module("multibody", "data");

    build_bridge_module("spatial", "se3");

    build_bridge_module("container", "joint_force_vector");

    build_bridge_module("algorithm", "joint_configuration");
    build_bridge_module("algorithm", "kinematics");
    build_bridge_module("algorithm", "frames");
    build_bridge_module("algorithm", "jacobian");
    build_bridge_module("algorithm", "rnea");
    build_bridge_module("algorithm", "aba");
    build_bridge_module("algorithm", "crba");
}