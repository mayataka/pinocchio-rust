use pinocchio as pin;
use nalgebra as na;

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
    let q = pin::random_configuration(&manipulator, &qmin, &qmax);
    println!("{:?}", q);
    let q = q.unwrap();
    let v = na::DVector::<f64>::zeros(manipulator.nv());
    let a = na::DVector::<f64>::zeros(manipulator.nv());
    println!("q: {}", q);
    let result = pin::frames_forward_kinematics(&manipulator, &mut manipulator_data, &q);

    let frame_translation = manipulator_data.frame_translation(7);
    let frame_translation = frame_translation.unwrap();
    println!("frame_translation: {}", frame_translation);
    let frame_rotation = manipulator_data.frame_rotation(7);
    let frame_rotation = frame_rotation.unwrap();
    println!("frame_rotation: {}", frame_rotation);

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