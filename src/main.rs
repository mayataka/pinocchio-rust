mod multibody;

pub use crate::multibody::model::*;
pub use crate::multibody::data::*;

fn main() {
    let mut model = Model::new();
    let urdf_path = "tests/anymal_b_simple_description/urdf/anymal.urdf";
    let base_joint_type = BaseJointType::FloatingBase;
    model.build_model_from_urdf(&urdf_path, &base_joint_type);
    let mut data = Data::new(&model);

    println!("main");
}