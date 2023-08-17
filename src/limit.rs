use serde::{Deserialize, Serialize};

#[derive(Debug, Deserialize, Serialize)]
/// Limit wrapper
pub struct Limit<T>
where
    T: Copy + Ord,
{
    /// lower limit
    pub min: T,
    /// upper limit
    pub max: T,
}

impl<T> Limit<T>
where
    T: Copy + Ord,
{
    /// Clamp value to limits
    pub fn clamp(&self, value: T) -> T {
        value.clamp(self.min, self.max)
    }
}
