use std::f64::{INFINITY, NAN};

use crate::{MotorController, Result, PID};

/// Fake motor implementation for testing purposes.
pub struct FakeMotor {
    torque_on: bool,

    current_position: f64,
    current_velocity: f64,
    current_torque: f64,

    target_position: f64,

    velocity_limit: f64,
    torque_limit: f64,
    pid: PID,
}

impl Default for FakeMotor {
    fn default() -> Self {
        Self {
            torque_on: false,

            current_position: 0.0,
            current_velocity: NAN,
            current_torque: NAN,

            target_position: 0.0,

            velocity_limit: INFINITY,
            torque_limit: INFINITY,
            pid: PID {
                p: NAN,
                i: NAN,
                d: NAN,
            },
        }
    }
}

impl MotorController for FakeMotor {
    fn is_torque_on(&self) -> Result<bool> {
        Ok(self.torque_on)
    }

    fn set_torque(&mut self, on: bool) -> Result<()> {
        if on != self.torque_on {
            self.current_position = self.target_position;
        }
        self.torque_on = on;
        Ok(())
    }

    fn get_current_position(&mut self) -> Result<f64> {
        Ok(self.current_position)
    }

    fn get_current_velocity(&mut self) -> Result<f64> {
        Ok(self.current_velocity)
    }

    fn get_current_torque(&mut self) -> Result<f64> {
        Ok(self.current_torque)
    }

    fn get_target_position(&mut self) -> Result<f64> {
        Ok(self.target_position)
    }

    fn set_target_position(&mut self, position: f64) -> Result<()> {
        self.target_position = position;

        if self.torque_on {
            self.current_position = position;
        }

        Ok(())
    }

    fn get_velocity_limit(&mut self) -> Result<f64> {
        Ok(self.velocity_limit)
    }

    fn set_velocity_limit(&mut self, velocity: f64) -> Result<()> {
        self.velocity_limit = velocity;
        Ok(())
    }

    fn get_torque_limit(&mut self) -> Result<f64> {
        Ok(self.torque_limit)
    }

    fn set_torque_limit(&mut self, torque: f64) -> Result<()> {
        self.torque_limit = torque;
        Ok(())
    }

    fn get_pid_gains(&mut self) -> Result<crate::PID> {
        Ok(self.pid)
    }

    fn set_pid_gains(&mut self, pid: crate::PID) -> Result<()> {
        self.pid = pid;
        Ok(())
    }
}