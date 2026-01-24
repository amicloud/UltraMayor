#[allow(dead_code)]
pub trait Action {
    fn execute(&mut self);
    fn undo(&mut self);
}

