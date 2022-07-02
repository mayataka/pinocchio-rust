use pinocchio as pin;
use nalgebra as na;
use rand::{thread_rng, Rng};

fn random_dvector(size: usize) -> na::DVector<f64> { 
    let mut v = vec![0f64; size];
    thread_rng().fill(&mut v[..]);
    na::DVector::from_vec(v)
}

fn run_example(model: &pin::Model, frame_id: usize) {
    let mut data = pin::Data::new(&model);

    let mut qmin = model.lower_position_limit();
    let mut qmax = model.upper_position_limit();
    println!("qmin: {}", qmin);
    println!("qmax: {}", qmax);
    if model.nq() != model.nv() { // if floating base
        for i in 0..6 {
            qmin[i] = -1.0;
            qmax[i] = 1.0;
        }
    }
    let qmin = qmin;
    let qmax = qmax;
    let q = pin::random_configuration(&model, &qmin, &qmax).unwrap();
    let v = random_dvector(model.nv());
    let a = random_dvector(model.nv());
    println!("q: {}", q);
    println!("v: {}", v);
    println!("a: {}", a);
    let result = pin::forward_kinematics_acceleration_level(&model, &mut data, &q, &v, &a);
    let result = pin::frames_forward_kinematics(&model, &mut data, &q);

    let frame_translation = data.frame_translation(frame_id).unwrap();
    println!("frame_translation: {}", frame_translation);
    let frame_rotation = data.frame_rotation(frame_id).unwrap();
    println!("frame_rotation: {}", frame_rotation);
    let rf = pin::ReferenceFrame::Local;
    let frame_velocity = pin::get_frame_velocity(&model, &data, frame_id, rf).unwrap();
    println!("frame_velocity: {}", frame_velocity);
    let rf = pin::ReferenceFrame::World;
    let frame_acceleration = pin::get_frame_acceleration(&model, &data, frame_id, rf).unwrap();
    println!("frame_acceleration: {}", frame_acceleration);
    let rf = pin::ReferenceFrame::LocalWorldAligned;
    let frame_classical_acceleration = pin::get_frame_classical_acceleration(&model, &data, frame_id, rf).unwrap();
    println!("frame_classical_acceleration: {}", frame_classical_acceleration);

    let result = pin::rnea(&model, &mut data, &q, &v, &a);
    let tau = data.tau().unwrap();
    println!("tau (rnea): {}", tau);

    let result = pin::aba(&model, &mut data, &q, &v, &tau);
    let ddq = data.ddq().unwrap();
    println!("ddq (aba): {}", ddq);

    let result = pin::crba(&model, &mut data, &q);
    let M = data.M().unwrap();
    println!("M (crba): {}", M);

    let f = pin::JointForceVector::new(model.njoints());

    let result = pin::rnea_with_external_forces(&model, &mut data, &q, &v, &a, &f);
    let tau = data.tau().unwrap();
    println!("tau (rnea with f): {}", tau);

    let result = pin::aba_with_external_forces(&model, &mut data, &q, &v, &tau, &f);
    let ddq = data.ddq().unwrap();
    println!("ddq (aba with f): {}", ddq);

    pin::compute_joint_jacobians(&model, &mut data, &q);
    let rf = pin::ReferenceFrame::LocalWorldAligned;
    let mut jac = na::DMatrix::<f64>::zeros(6, model.nv());
    pin::get_joint_jacobian(&model, &mut data, 6, rf, &mut jac);
    println!("jac: {}", jac);
}


fn main() {
    let mut manipulator = pin::Model::new();
    manipulator.build_sample_manipulator();
    println!("Sample manipulator: {}", manipulator);
    let frame_id = 7;
    run_example(&manipulator, frame_id);

    let mut humanoid = pin::Model::new();
    humanoid.build_sample_humanoid();
    println!("Sample humanoid: {}", humanoid);
    let frame_id = 12;
    run_example(&humanoid, frame_id);

    let mut anymal = pin::Model::new();
    let urdf_path = "tests/anymal_b_simple_description/urdf/anymal.urdf";
    let base_joint_type = pin::BaseJointType::FloatingBase;
    anymal.build_model_from_urdf(&urdf_path, base_joint_type);
    println!("ANYmal: {}", anymal);
    let frame_id = anymal.frame_id("LF_FOOT").unwrap();
    run_example(&anymal, frame_id);
}