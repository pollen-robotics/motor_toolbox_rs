use std::f64::{INFINITY, NAN};

use itertools::izip;

use crate::motors_controller::MotorsController;
use crate::motors_io::RawMotorsIO;
use crate::{Limit, Result, PID};

#[derive(Debug)]
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

#[derive(Debug)]
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
        log::info!(target: "fake_io::set_torque", "Setting torque to {:?}", on);

        for (cur, target, on, torque_on) in izip!(
            &mut self.current_position,
            self.target_position,
            on,
            self.torque_on
        ) {
            if on && !torque_on {
                log::debug!(target: "fake_io::set_torque", "Setting current position to target position {:?}", target);
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
        log::info!(target: "fake_io::set_target_position", "Setting target_position to {:?}", target_position);
        self.target_position = target_position;

        for (cur, on, target) in izip!(&mut self.current_position, self.torque_on, target_position)
        {
            if on {
                log::debug!(target: "fake_io::set_target_position", "Setting current position to target position {:?} (torque on)", target);
                *cur = target;
            } else {
                log::debug!(target: "fake_io::set_target_position", "Current position unchanged (torque off)");
            }
        }

        Ok(())
    }

    fn set_target_position_fb(&mut self, target_position: [f64; N]) -> Result<[f64; N]> {
        log::info!(target: "fake_io::set_target_position", "Setting target_position to {:?}", target_position);
        self.target_position = target_position;
        let mut fb: [f64; N] = [0.0; { N }];

        for (cur, on, target) in izip!(&mut self.current_position, self.torque_on, target_position)
        {
            if on {
                log::debug!(target: "fake_io::set_target_position", "Setting current position to target position {:?} (torque on)", target);
                *cur = target;
            } else {
                log::debug!(target: "fake_io::set_target_position", "Current position unchanged (torque off)");
            }
        }
        fb[0..N].copy_from_slice(&self.current_position);
        // fb[N..2*N].copy_from_slice(&self.current_velocity);
        // fb[2*N..3*N].copy_from_slice(&self.current_torque);

        Ok(fb)
    }

    fn get_velocity_limit(&mut self) -> Result<[f64; N]> {
        Ok(self.velocity_limit)
    }

    fn set_velocity_limit(&mut self, velocity: [f64; N]) -> Result<()> {
        log::info!(target: "fake_io::set_velocity_limit", "Setting velocity_limit to {:?}", velocity);
        self.velocity_limit = velocity;
        Ok(())
    }

    fn get_torque_limit(&mut self) -> Result<[f64; N]> {
        Ok(self.torque_limit)
    }

    fn set_torque_limit(&mut self, torque: [f64; N]) -> Result<()> {
        log::info!(target: "fake_io::set_torque_limit", "Setting torque_limit to {:?}", torque);
        self.torque_limit = torque;
        Ok(())
    }

    fn get_pid_gains(&mut self) -> Result<[PID; N]> {
        Ok(self.pid)
    }

    fn set_pid_gains(&mut self, pid: [PID; N]) -> Result<()> {
        log::info!(target: "fake_io::set_pid_gains", "Setting pid gains to {:?}", pid);
        self.pid = pid;
        Ok(())
    }

    fn get_axis_sensors(&mut self) -> Result<[f64; N]> {
        Ok(self.current_position)
    }

    fn get_board_state(&mut self) -> Result<u8> {
        Ok(0)
    }
    fn set_board_state(&mut self, _state: u8) -> Result<()> {
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    mod controller {
        use std::f64::consts::PI;

        use crate::fake_motor::FakeMotorsController;
        use crate::motors_controller::MotorsController;
        use crate::PID;

        #[test]
        fn check_default() {
            let motor = FakeMotorsController::<1>::default();

            assert_eq!(motor.offsets(), [None]);
            assert_eq!(motor.reduction(), [None]);
            assert_eq!(motor.limits(), [None]);
        }

        #[test]
        fn check_with() {
            let motor = FakeMotorsController::<1>::new();

            assert_eq!(motor.offsets(), [None]);
            assert_eq!(motor.reduction(), [None]);
            assert_eq!(motor.limits(), [None]);

            let motor = motor.with_offsets([Some(1.0)]);
            assert_eq!(motor.offsets(), [Some(1.0)]);
            assert_eq!(motor.reduction(), [None]);
            assert_eq!(motor.limits(), [None]);

            let motor = motor.with_reduction([Some(2.0)]);
            assert_eq!(motor.offsets(), [Some(1.0)]);
            assert_eq!(motor.reduction(), [Some(2.0)]);
            assert_eq!(motor.limits(), [None]);

            let motor = motor.with_limits([Some((0.0, 1.0).try_into().unwrap())]);
            assert_eq!(motor.offsets(), [Some(1.0)]);
            assert_eq!(motor.reduction(), [Some(2.0)]);
            assert_eq!(motor.limits(), [Some((0.0, 1.0).try_into().unwrap())]);

            let motor = FakeMotorsController::<1>::new()
                .with_offsets([Some(1.0)])
                .with_limits([Some((0.0, 1.0).try_into().unwrap())]);
            assert_eq!(motor.offsets(), [Some(1.0)]);
            assert_eq!(motor.reduction(), [None]);
            assert_eq!(motor.limits(), [Some((0.0, 1.0).try_into().unwrap())]);
        }

        #[test]
        fn check_torque() {
            let mut motor = FakeMotorsController::<3>::default();

            let torques = motor.is_torque_on().unwrap();
            assert!(torques.iter().all(|&x| !x));

            motor.set_torque([true, false, true]).unwrap();
            let torques = motor.is_torque_on().unwrap();
            assert!(torques[0]);
            assert!(!torques[1]);
            assert!(torques[2]);

            motor.set_torque([false, true, false]).unwrap();
            let torques = motor.is_torque_on().unwrap();
            assert!(!torques[0]);
            assert!(torques[1]);
            assert!(!torques[2]);
        }

        #[test]
        fn offset() {
            let mut motor =
                FakeMotorsController::<3>::new().with_offsets([Some(-1.0), Some(1.0), None]);

            motor.set_torque([true; 3]).unwrap();
            assert_eq!(motor.get_current_position().unwrap(), [1.0, -1.0, 0.0]);

            motor.set_target_position([0.0, 0.0, 0.0]).unwrap();
            assert_eq!(motor.get_target_position().unwrap(), [0.0, 0.0, 0.0]);
            assert_eq!(motor.get_current_position().unwrap(), [0.0, 0.0, 0.0]);

            let raw_current = motor.io().get_current_position().unwrap();
            assert_eq!(raw_current, [-1.0, 1.0, 0.0]);
            let raw_target = motor.io().get_target_position().unwrap();
            assert_eq!(raw_target, [-1.0, 1.0, 0.0]);
        }

        #[test]
        fn reduction() {
            let mut motor =
                FakeMotorsController::<3>::new().with_reduction([Some(-2.0), None, Some(1.0)]);

            motor.set_torque([true; 3]).unwrap();
            assert_eq!(motor.get_current_position().unwrap(), [0.0, 0.0, 0.0]);

            motor.set_target_position([1.0, 1.0, 1.0]).unwrap();
            assert_eq!(motor.get_target_position().unwrap(), [1.0, 1.0, 1.0]);
            assert_eq!(motor.get_current_position().unwrap(), [1.0, 1.0, 1.0]);

            let raw_current = motor.io().get_current_position().unwrap();
            assert_eq!(raw_current, [-2.0, 1.0, 1.0]);
            let raw_target = motor.io().get_target_position().unwrap();
            assert_eq!(raw_target, [-2.0, 1.0, 1.0]);
        }

        #[test]
        fn limit() {
            let mut motor = FakeMotorsController::<3>::new().with_limits([
                Some((0.0, 1.0).try_into().unwrap()),
                Some((-1.0, 1.0).try_into().unwrap()),
                None,
            ]);

            motor.set_torque([true; 3]).unwrap();
            assert_eq!(motor.get_current_position().unwrap(), [0.0, 0.0, 0.0]);

            motor.set_target_position([-0.5, -0.5, -0.5]).unwrap();
            assert_eq!(motor.get_target_position().unwrap(), [0.0, -0.5, -0.5]);
            assert_eq!(motor.get_current_position().unwrap(), [0.0, -0.5, -0.5]);

            let raw_current = motor.io().get_current_position().unwrap();
            assert_eq!(raw_current, [0.0, -0.5, -0.5]);
            let raw_target = motor.io().get_target_position().unwrap();
            assert_eq!(raw_target, [0.0, -0.5, -0.5]);

            motor.set_target_position([PI, PI, PI]).unwrap();
            assert_eq!(motor.get_target_position().unwrap(), [1.0, 1.0, PI]);
            assert_eq!(motor.get_current_position().unwrap(), [1.0, 1.0, PI]);

            let raw_current = motor.io().get_current_position().unwrap();
            assert_eq!(raw_current, [1.0, 1.0, PI]);
            let raw_target = motor.io().get_target_position().unwrap();
            assert_eq!(raw_target, [1.0, 1.0, PI]);
        }

        #[test]
        fn offset_reduction_limit() {
            let mut motor = FakeMotorsController::<3>::new()
                .with_offsets([Some(-1.0), Some(1.0), None])
                .with_reduction([Some(2.0), None, Some(1.0)])
                .with_limits([
                    Some((0.0, 1.0).try_into().unwrap()),
                    Some((-1.0, 1.0).try_into().unwrap()),
                    None,
                ]);

            motor.set_torque([true; 3]).unwrap();
            assert_eq!(motor.get_current_position().unwrap(), [1.0, -1.0, 0.0]);

            motor.set_target_position([0.0, 0.0, 0.0]).unwrap();
            assert_eq!(motor.get_target_position().unwrap(), [0.0, 0.0, 0.0]);
            assert_eq!(motor.get_current_position().unwrap(), [0.0, 0.0, 0.0]);

            motor.set_target_position([-0.5, -0.5, -0.5]).unwrap();
            assert_eq!(motor.get_target_position().unwrap(), [0.0, -0.5, -0.5]);
            assert_eq!(motor.get_current_position().unwrap(), [0.0, -0.5, -0.5]);

            let raw_current = motor.io().get_current_position().unwrap();
            assert_eq!(raw_current, [-2.0, 0.5, -0.5]);

            motor.set_target_position([PI, PI, PI]).unwrap();
            assert_eq!(motor.get_target_position().unwrap(), [1.0, 1.0, PI]);
            assert_eq!(motor.get_current_position().unwrap(), [1.0, 1.0, PI]);

            let raw_current = motor.io().get_current_position().unwrap();
            assert_eq!(raw_current, [0.0, 2.0, PI]);
            let raw_target = motor.io().get_target_position().unwrap();
            assert_eq!(raw_target, [0.0, 2.0, PI]);
        }

        #[test]
        fn target_position() {
            let mut motors = FakeMotorsController::<3>::new()
                .with_offsets([Some(-1.0), Some(1.0), None])
                .with_reduction([Some(2.0), None, Some(1.0)])
                .with_limits([
                    Some((0.0, 1.0).try_into().unwrap()),
                    Some((-1.0, 1.0).try_into().unwrap()),
                    None,
                ]);

            motors.set_target_position([0.0, 1.0, 2.0]).unwrap();
            assert_eq!(motors.get_target_position().unwrap(), [0.0, 1.0, 2.0]);

            motors.set_target_position([-1.0, 0.0, 1.0]).unwrap();
            assert_eq!(motors.get_target_position().unwrap(), [0.0, 0.0, 1.0]);
        }

        #[test]
        fn velocity_limit() {
            let mut motors = FakeMotorsController::<3>::new()
                .with_offsets([Some(-1.0), Some(1.0), None])
                .with_reduction([Some(2.0), None, Some(1.0)])
                .with_limits([
                    Some((0.0, 1.0).try_into().unwrap()),
                    Some((-1.0, 1.0).try_into().unwrap()),
                    None,
                ]);

            motors.set_velocity_limit([1.0, 2.0, 3.0]).unwrap();
            assert_eq!(motors.get_velocity_limit().unwrap(), [1.0, 2.0, 3.0]);
        }

        #[test]
        fn torque_limit() {
            let mut motors = FakeMotorsController::<3>::new()
                .with_offsets([Some(-1.0), Some(1.0), None])
                .with_reduction([Some(2.0), None, Some(1.0)])
                .with_limits([
                    Some((0.0, 1.0).try_into().unwrap()),
                    Some((-1.0, 1.0).try_into().unwrap()),
                    None,
                ]);

            motors.set_torque_limit([1.0, 2.0, 3.0]).unwrap();
            assert_eq!(motors.get_torque_limit().unwrap(), [1.0, 2.0, 3.0]);
        }

        #[test]
        fn pid() {
            let mut motors = FakeMotorsController::<3>::new()
                .with_offsets([Some(-1.0), Some(1.0), None])
                .with_reduction([Some(2.0), None, Some(1.0)])
                .with_limits([
                    Some((0.0, 1.0).try_into().unwrap()),
                    Some((-1.0, 1.0).try_into().unwrap()),
                    None,
                ]);

            let pids = [
                PID {
                    p: 1.0,
                    i: 2.0,
                    d: 3.0,
                },
                PID {
                    p: 4.0,
                    i: 5.0,
                    d: 6.0,
                },
                PID {
                    p: 7.0,
                    i: 8.0,
                    d: 9.0,
                },
            ];
            motors.set_pid_gains(pids).unwrap();
            assert_eq!(motors.get_pid_gains().unwrap(), pids);
        }
    }

    mod io {
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
}
