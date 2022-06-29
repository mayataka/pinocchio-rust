mod multibody;
mod algorithm;
mod math;
pub use crate::multibody::model::*;
pub use crate::multibody::data::*;
pub use crate::algorithm::frames::*;
pub use crate::algorithm::rnea::*;
pub use crate::algorithm::aba::*;

fn main() {
    // let mut model = Model::new();
    // let urdf_path = "tests/anymal_b_simple_description/urdf/anymal.urdf";
    // let base_joint_type = BaseJointType::FloatingBase;
    // model.build_model_from_urdf(&urdf_path, &base_joint_type);
    // let mut data = Data::new(&model);

    let mut manipulator = Model::new();
    manipulator.build_sample_manipulator();
    println!("{}", manipulator);
    let mut manipulator_data = Data::new(&manipulator);

    let mut humanoid = Model::new();
    humanoid.build_sample_humanoid();
    println!("{}", humanoid);
    let mut humanoid_data = Data::new(&humanoid);

    let frame_translation = humanoid_data.frame_translation(1);
    println!("{}", frame_translation.unwrap());

    let frame_rotation= humanoid_data.frame_rotation(1);
    println!("{}", frame_rotation.unwrap());

    // let frame_placement = humanoid_data.frame_placement(1);
    // println!("{}", frame_placement.unwrap());
}