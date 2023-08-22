use serde::{Deserialize, Serialize};

#[derive(Clone, Copy, Debug, PartialEq, Deserialize, Serialize)]
/// Limit wrapper
pub struct Limit {
    /// lower limit
    min: f64,
    /// upper limit
    max: f64,
}

impl Limit {
    /// Create new limit
    pub fn new(min: f64, max: f64) -> Self {
        if min > max {
            panic!("min must be less than max");
        }
        Self { min, max }
    }

    /// lower limit
    pub fn min(&self) -> f64 {
        self.min
    }

    /// upper limit
    pub fn max(&self) -> f64 {
        self.max
    }

    /// Clamp value to limits
    pub fn clamp(&self, value: f64) -> f64 {
        value.clamp(self.min, self.max)
    }
}

#[cfg(test)]
mod tests {
    use crate::Limit;

    #[test]
    #[should_panic]
    fn bad_limit() {
        Limit::new(1.0, -1.0);
    }

    #[test]
    fn check_limit() {
        let limit = Limit::new(-1.0, 1.0);

        assert_eq!(limit.clamp(-2.0), -1.0);
        assert_eq!(limit.clamp(0.0), 0.0);
        assert_eq!(limit.clamp(2.0), 1.0);
    }

    #[test]
    fn test_eq() {
        let limit1 = Limit::new(-1.0, 1.0);
        let limit2 = Limit::new(-1.0, 1.0);
        let limit3 = Limit::new(-1.0, 2.0);
        let limit4 = Limit::new(0.0, 1.0);

        assert_eq!(limit1, limit2);
        assert_ne!(limit1, limit3);
        assert_ne!(limit1, limit4);
    }
}
