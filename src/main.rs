mod multibody;

pub use crate::multibody::model::*;
pub use crate::multibody::data::*;

fn main() {
    // let mut model = Model::new();
    // let urdf_path = "tests/anymal_b_simple_description/urdf/anymal.urdf";
    // let base_joint_type = BaseJointType::FloatingBase;
    // model.build_model_from_urdf(&urdf_path, &base_joint_type);
    // let mut data = Data::new(&model);

    let mut manipulator = Model::new();
    manipulator.build_sample_manipulator();
    let mut manipulator_data = Data::new(&manipulator);
    println!("{}", manipulator);

    let mut humanoid = Model::new();
    humanoid.build_sample_humanoid();
    let mut humanoid_data = Data::new(&humanoid);
    println!("{}", humanoid);

    let frame_placement = humanoid_data.frame_placement(1);
    println!("{}", frame_placement.unwrap());
}