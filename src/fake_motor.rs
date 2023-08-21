use std::f64::{INFINITY, NAN};

use itertools::izip;

use crate::motors_controller::MotorsController;
use crate::motors_io::RawMotorsIO;
use crate::{Limit, Result, PID};

pub struct FakeMotorsController<const N: usize> {
    offsets: [Option<f64>; N],
    reduction: [Option<f64>; N],
    limits: [Option<Limit>; N],

    io: FakeMotorsIO<N>,
}

impl<const N: usize> FakeMotorsController<N> {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn with_offsets(mut self, offsets: [Option<f64>; N]) -> Self {
        self.offsets = offsets;
        self
    }

    pub fn with_reduction(mut self, reduction: [Option<f64>; N]) -> Self {
        self.reduction = reduction;
        self
    }

    pub fn with_limits(mut self, limits: [Option<Limit>; N]) -> Self {
        self.limits = limits;
        self
    }
}

impl<const N: usize> Default for FakeMotorsController<N> {
    fn default() -> Self {
        Self {
            offsets: [None; N],
            reduction: [None; N],
            limits: [None; N],

            io: FakeMotorsIO::<N>::default(),
        }
    }
}

impl<const N: usize> MotorsController<N> for FakeMotorsController<N> {
    fn offsets(&self) -> [Option<f64>; N] {
        self.offsets
    }

    fn reduction(&self) -> [Option<f64>; N] {
        self.reduction
    }

    fn limits(&self) -> [Option<Limit>; N] {
        self.limits
    }

    fn io(&mut self) -> &mut dyn RawMotorsIO<N> {
        &mut self.io
    }
}

/// Fake motor io implementation for testing purposes.
pub struct FakeMotorsIO<const N: usize> {
    torque_on: [bool; N],

    current_position: [f64; N],
    current_velocity: [f64; N],
    current_torque: [f64; N],

    target_position: [f64; N],

    velocity_limit: [f64; N],
    torque_limit: [f64; N],
    pid: [PID; N],
}

impl<const N: usize> Default for FakeMotorsIO<N> {
    fn default() -> Self {
        Self {
            torque_on: [false; N],

            current_position: [0.0; N],
            current_velocity: [NAN; N],
            current_torque: [NAN; N],

            target_position: [0.0; N],

            velocity_limit: [INFINITY; N],
            torque_limit: [INFINITY; N],
            pid: [PID {
                p: NAN,
                i: NAN,
                d: NAN,
            }; N],
        }
    }
}

impl<const N: usize> RawMotorsIO<N> for FakeMotorsIO<N> {
    fn is_torque_on(&mut self) -> Result<[bool; N]> {
        Ok(self.torque_on)
    }

    fn set_torque(&mut self, on: [bool; N]) -> Result<()> {
        for (cur, target, on, torque_on) in izip!(
            &mut self.current_position,
            self.target_position,
            on,
            self.torque_on
        ) {
            if on && !torque_on {
                *cur = target;
            }
        }

        self.torque_on = on;

        Ok(())
    }

    fn get_current_position(&mut self) -> Result<[f64; N]> {
        Ok(self.current_position)
    }

    fn get_current_velocity(&mut self) -> Result<[f64; N]> {
        Ok(self.current_velocity)
    }

    fn get_current_torque(&mut self) -> Result<[f64; N]> {
        Ok(self.current_torque)
    }

    fn get_target_position(&mut self) -> Result<[f64; N]> {
        Ok(self.target_position)
    }

    fn set_target_position(&mut self, target_position: [f64; N]) -> Result<()> {
        self.target_position = target_position;

        for (cur, on, targ) in izip!(&mut self.current_position, self.torque_on, target_position) {
            if on {
                *cur = targ;
            }
        }

        Ok(())
    }

    fn get_velocity_limit(&mut self) -> Result<[f64; N]> {
        Ok(self.velocity_limit)
    }

    fn set_velocity_limit(&mut self, velocity: [f64; N]) -> Result<()> {
        self.velocity_limit = velocity;
        Ok(())
    }

    fn get_torque_limit(&mut self) -> Result<[f64; N]> {
        Ok(self.torque_limit)
    }

    fn set_torque_limit(&mut self, torque: [f64; N]) -> Result<()> {
        self.torque_limit = torque;
        Ok(())
    }

    fn get_pid_gains(&mut self) -> Result<[PID; N]> {
        Ok(self.pid)
    }

    fn set_pid_gains(&mut self, pid: [PID; N]) -> Result<()> {
        self.pid = pid;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use crate::{fake_motor::FakeMotorsIO, motors_io::RawMotorsIO};

    #[test]
    fn check_default() {
        let mut motor = FakeMotorsIO::<1>::default();

        assert!(!motor.is_torque_on().unwrap()[0]);
        assert_eq!(motor.get_current_position().unwrap(), [0.0]);
        assert_eq!(motor.get_target_position().unwrap(), [0.0]);
    }

    #[test]
    fn set_target() {
        let mut motor = FakeMotorsIO::<1>::default();

        // With torque off, the current position should not change
        motor.set_target_position([0.5]).unwrap();
        assert_eq!(motor.get_target_position().unwrap(), [0.5]);
        assert_eq!(motor.get_current_position().unwrap(), [0.0]);

        // Enabling the torque
        motor.set_torque([true]).unwrap();
        assert!(motor.is_torque_on().unwrap()[0]);
        assert_eq!(motor.get_current_position().unwrap(), [0.5]);
        assert_eq!(motor.get_target_position().unwrap(), [0.5]);

        // Setting the target position
        motor.set_target_position([0.25]).unwrap();
        assert_eq!(motor.get_target_position().unwrap(), [0.25]);
        assert_eq!(motor.get_current_position().unwrap(), [0.25]);
    }

    #[test]
    fn multiple_fake() {
        let mut motors = FakeMotorsIO::<3>::default();
        motors.set_torque([true, false, true]).unwrap();
        assert_eq!(motors.is_torque_on().unwrap(), [true, false, true]);
    }
}
