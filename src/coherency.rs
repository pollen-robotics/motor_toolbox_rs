use crate::motor_controller::Result;

pub trait CoherentResult<T> {
    fn coherent(self) -> Result<T>;
}

#[derive(Debug)]
struct IncoherentError;
impl std::fmt::Display for IncoherentError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "(incoherent values)",)
    }
}
impl std::error::Error for IncoherentError {}

impl<T, U: Iterator<Item = Result<T>>> CoherentResult<T> for U
where
    T: Copy + Clone + PartialEq + std::fmt::Debug,
{
    fn coherent(self) -> Result<T> {
        let mut iter = self;

        if let Some(Ok(first)) = iter.next() {
            for x in iter {
                match x {
                    Ok(x) => {
                        if x != first {
                            return Err(Box::new(IncoherentError));
                        }
                    }
                    Err(e) => return Err(e),
                }
            }
            Ok(first)
        } else {
            Err(Box::new(IncoherentError))
        }
    }
}
