use pinocchio as pin;
use nalgebra as na;
use rand::{thread_rng, Rng};

fn random_dvector(size: usize) -> na::DVector<f64> { 
    let mut v = vec![0f64; size];
    thread_rng().fill(&mut v[..]);
    na::DVector::from_vec(v)
}

fn main() {
    // let mut model = pin::Model::new();
    // let urdf_path = "tests/anymal_b_simple_description/urdf/anymal.urdf";
    // let base_joint_type = pin::BaseJointType::FloatingBase;
    // model.build_model_from_urdf(&urdf_path, base_joint_type);
    // let mut data = pin::Data::new(&model);

    let mut manipulator = pin::Model::new();
    manipulator.build_sample_manipulator();
    println!("{}", manipulator);
    let qmin = manipulator.lower_position_limit();
    let qmax = manipulator.upper_position_limit();
    let mut manipulator_data = pin::Data::new(&manipulator);

    // let mut humanoid = pin::Model::new();
    // humanoid.build_sample_humanoid();
    // println!("{}", humanoid);
    // let mut humanoid_data = pin::Data::new(&humanoid);

    println!("qmin: {}", qmin);
    println!("qmax: {}", qmax);
    let q = pin::random_configuration(&manipulator, &qmin, &qmax).unwrap();
    let v = random_dvector(manipulator.nv());
    let a = random_dvector(manipulator.nv());
    println!("q: {}", q);
    println!("v: {}", v);
    println!("a: {}", a);
    let result = pin::forward_kinematics_acceleration_level(&manipulator, &mut manipulator_data, &q, &v, &a);
    let result = pin::frames_forward_kinematics(&manipulator, &mut manipulator_data, &q);

    let frame_translation = manipulator_data.frame_translation(6);
    let frame_translation = frame_translation.unwrap();
    println!("frame_translation: {}", frame_translation);
    let frame_rotation = manipulator_data.frame_rotation(6);
    let frame_rotation = frame_rotation.unwrap();
    println!("frame_rotation: {}", frame_rotation);
    let frame_velocity = pin::get_frame_velocity(&manipulator, &manipulator_data, 6).unwrap();
    println!("frame_velocity: {}", frame_velocity);
    let frame_acceleration = pin::get_frame_acceleration(&manipulator, &manipulator_data, 6).unwrap();
    println!("frame_acceleration: {}", frame_acceleration);
    let frame_classical_acceleration = pin::get_frame_classical_acceleration(&manipulator, &manipulator_data, 6).unwrap();
    println!("frame_classical_acceleration: {}", frame_classical_acceleration);

    let result = pin::rnea(&manipulator, &mut manipulator_data, &q, &v, &a);
    let tau = manipulator_data.tau();
    let tau = tau.unwrap();
    println!("tau (rnea): {}", tau);

    let result = pin::aba(&manipulator, &mut manipulator_data, &q, &v, &tau);
    let ddq = manipulator_data.ddq();
    let ddq = ddq.unwrap();
    println!("ddq (aba): {}", ddq);

    let result = pin::crba(&manipulator, &mut manipulator_data, &q);
    let M = manipulator_data.M();
    let M = M.unwrap();
    println!("M (crba): {}", M);

}