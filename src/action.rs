use std::{cell::RefCell, rc::Rc};

use nalgebra::{Quaternion, Vector3};

use crate::body::Body;

pub trait Action {
    fn execute(&mut self);
    fn undo(&mut self);
}

pub struct SetPositionAction {
    pub body: Rc<RefCell<Body>>,
    pub input: Vector3<f32>,
    pub previous: Vector3<f32>,
}

impl Action for SetPositionAction {
    fn execute(&mut self) {
        self.body.borrow_mut().set_position(self.input);
    }

    fn undo(&mut self) {
        self.body.borrow_mut().set_position(self.previous);
    }
}

pub struct SetRotationAction {
    pub body: Rc<RefCell<Body>>,
    pub input: Vector3<f32>,
    pub previous: Quaternion<f32>,
}

impl Action for SetRotationAction {
    fn execute(&mut self) {
        self.body.borrow_mut().set_rotation(self.input);
    }

    fn undo(&mut self) {
        self.body.borrow_mut().set_rotation_quat(self.previous);
    }
}

pub struct SetScaleAction {
    pub body: Rc<RefCell<Body>>,
    pub input: Vector3<f32>,
    pub previous: Vector3<f32>,
}

impl Action for SetScaleAction {
    fn execute(&mut self) {
        self.body.borrow_mut().set_scale(self.input);
    }

    fn undo(&mut self) {
        self.body.borrow_mut().set_scale(self.previous);
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::{Quaternion, Vector3};
    use std::{cell::RefCell, rc::Rc};

    // Helper function to compare Vector3<f32> with a tolerance
    fn assert_vectors_approx_equal(v1: &Vector3<f32>, v2: &Vector3<f32>) {
        let epsilon = 1e-5;
        assert!(
            (v1.x - v2.x).abs() < epsilon,
            "X components differ: {} vs {}",
            v1.x,
            v2.x
        );
        assert!(
            (v1.y - v2.y).abs() < epsilon,
            "Y components differ: {} vs {}",
            v1.y,
            v2.y
        );
        assert!(
            (v1.z - v2.z).abs() < epsilon,
            "Z components differ: {} vs {}",
            v1.z,
            v2.z
        );
    }

    // Helper function to compare Quaternions with a tolerance
    fn assert_quaternions_approx_equal(q1: &Quaternion<f32>, q2: &Quaternion<f32>) {
        let epsilon = 1e-5;
        assert!(
            (q1.w - q2.w).abs() < epsilon,
            "W components differ: {} vs {}",
            q1.w,
            q2.w
        );
        assert!(
            (q1.i - q2.i).abs() < epsilon,
            "I components differ: {} vs {}",
            q1.i,
            q2.i
        );
        assert!(
            (q1.j - q2.j).abs() < epsilon,
            "J components differ: {} vs {}",
            q1.j,
            q2.j
        );
        assert!(
            (q1.k - q2.k).abs() < epsilon,
            "K components differ: {} vs {}",
            q1.k,
            q2.k
        );
    }

    #[test]
    fn test_set_position_action() {
        // Create a new Body instance with default values
        let body = Rc::new(RefCell::new(Body::default()));

        // Record the previous position
        let previous_position = body.borrow().position;

        // Define the new position to set
        let new_position = Vector3::new(1.0, 2.0, 3.0);

        // Create the SetPositionAction
        let mut action = SetPositionAction {
            body: Rc::clone(&body),
            input: new_position,
            previous: previous_position,
        };

        // Execute the action
        action.execute();

        // Verify that the body's position has been updated
        assert_vectors_approx_equal(&body.borrow().position, &new_position);

        // Undo the action
        action.undo();

        // Verify that the body's position has been restored
        assert_vectors_approx_equal(&body.borrow().position, &previous_position);
    }

    #[test]
    fn test_set_rotation_action() {
        // Create a new Body instance with default values
        let body = Rc::new(RefCell::new(Body::default()));

        // Record the previous rotation
        let previous_rotation = body.borrow().rotation;

        // Define the new rotation in Euler angles
        let new_rotation = Vector3::new(45.0, 30.0, 60.0);

        // Create the SetRotationAction
        let mut action = SetRotationAction {
            body: Rc::clone(&body),
            input: new_rotation,
            previous: previous_rotation,
        };

        // Execute the action
        action.execute();

        // Calculate the expected quaternion from Euler angles
        let expected_quaternion = Body::euler_to_quaternion(new_rotation);

        // Verify that the body's rotation has been updated
        assert_quaternions_approx_equal(&body.borrow().rotation, &expected_quaternion);

        // Undo the action
        action.undo();

        // Verify that the body's rotation has been restored
        assert_quaternions_approx_equal(&body.borrow().rotation, &previous_rotation);
    }

    #[test]
    fn test_set_scale_action() {
        // Create a new Body instance with default values
        let body = Rc::new(RefCell::new(Body::default()));

        // Record the previous scale
        let previous_scale = body.borrow().scale;

        // Define the new scale to set
        let new_scale = Vector3::new(2.0, 2.0, 2.0);

        // Create the SetScaleAction
        let mut action = SetScaleAction {
            body: Rc::clone(&body),
            input: new_scale,
            previous: previous_scale,
        };

        // Execute the action
        action.execute();

        // Verify that the body's scale has been updated
        assert_vectors_approx_equal(&body.borrow().scale, &new_scale);

        // Undo the action
        action.undo();

        // Verify that the body's scale has been restored
        assert_vectors_approx_equal(&body.borrow().scale, &previous_scale);
    }
}
