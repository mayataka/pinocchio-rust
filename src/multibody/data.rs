use cxx::UniquePtr;
use crate::multibody::model::pinocchio::Model;

#[cxx::bridge(namespace = "pinocchio")]
pub mod pinocchio {
    unsafe extern "C++" {
        include!("pinocchio-rust/multibody/data.hpp");

        type Model = crate::multibody::pinocchio::Model;
        pub type Data;
        pub fn create_data(model: &UniquePtr<Model>) -> UniquePtr<Data>;
        pub fn clone(data : &Data) -> UniquePtr<Data>;
    }
}
