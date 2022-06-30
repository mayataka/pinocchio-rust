mod multibody;
mod algorithm;
mod math;
pub use crate::multibody::model::*;
pub use crate::multibody::data::*;
pub use crate::algorithm::frames::*;
pub use crate::algorithm::rnea::*;
pub use crate::algorithm::aba::*;

use nalgebra as na;
use std::vec::Vec;

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

    let q = na::DVector::zeros(manipulator.nq());
    let v = na::DVector::zeros(manipulator.nv());
    let a = na::DVector::zeros(manipulator.nv());
    let result = frames_forward_kinematics(&manipulator, &mut manipulator_data, &q);

    let frame_translation = manipulator_data.frame_translation(7);
    println!("{}", frame_translation.unwrap());
    let frame_rotation= manipulator_data.frame_rotation(7);
    println!("{}", frame_rotation.unwrap());

    let result = rnea(&manipulator, &mut manipulator_data, &q, &v, &a);
    let tau = manipulator_data.tau();
    let tau = tau.unwrap();
    println!("{}", tau);

    let result = aba(&manipulator, &mut manipulator_data, &q, &v, &tau);
    let ddq= manipulator_data.ddq();
    let ddq = ddq.unwrap();
    println!("{}", ddq);

}