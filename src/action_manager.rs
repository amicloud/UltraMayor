use crate::action::Action;

pub struct ActionManager {
    history: Vec<Box<dyn Action>>,
    future: Vec<Box<dyn Action>>,
}

impl ActionManager {
    pub fn new() -> Self {
        Self {
            history: Vec::new(),
            future: Vec::new(),
        }
    }

    pub fn execute(&mut self, mut action: Box<dyn Action>) {
        action.execute();
        self.history.push(action);
        self.future.clear(); // Clear the redo stack on new action
    }

    pub fn undo(&mut self) {
        if let Some(mut action) = self.history.pop() {
            action.undo();
            self.future.push(action);
        }
    }

    pub fn redo(&mut self) {
        if let Some(mut action) = self.future.pop() {
            action.execute();
            self.history.push(action);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::{Arc, Mutex};

    /// A mock implementation of the Action trait for testing purposes.
    struct MockAction {
        execute_called: Arc<Mutex<bool>>,
        undo_called: Arc<Mutex<bool>>,
    }

    impl MockAction {
        /// Creates a new MockAction instance.
        fn new() -> Self {
            Self {
                execute_called: Arc::new(Mutex::new(false)),
                undo_called: Arc::new(Mutex::new(false)),
            }
        }

        /// Returns a clone of the execute_called Arc for external access.
        fn execute_called(&self) -> Arc<Mutex<bool>> {
            Arc::clone(&self.execute_called)
        }

        /// Returns a clone of the undo_called Arc for external access.
        fn undo_called(&self) -> Arc<Mutex<bool>> {
            Arc::clone(&self.undo_called)
        }
    }

    impl Action for MockAction {
        fn execute(&mut self) {
            let mut exec = self.execute_called.lock().unwrap();
            *exec = true;
        }

        fn undo(&mut self) {
            let mut undo = self.undo_called.lock().unwrap();
            *undo = true;
        }
    }

    #[test]
    fn test_execute_adds_action_to_history() {
        let mut manager = ActionManager::new();
        let action = Box::new(MockAction::new());

        manager.execute(action);

        assert_eq!(manager.history.len(), 1);
        assert_eq!(manager.future.len(), 0);
    }

    #[test]
    fn test_execute_clears_future() {
        let mut manager = ActionManager::new();

        // Execute first action
        let action1 = Box::new(MockAction::new());
        manager.execute(action1);

        // Undo the first action
        manager.undo();

        assert_eq!(manager.history.len(), 0);
        assert_eq!(manager.future.len(), 1);

        // Execute a new action, which should clear the future
        let action2 = Box::new(MockAction::new());
        manager.execute(action2);

        assert_eq!(manager.history.len(), 1);
        assert_eq!(manager.future.len(), 0);
    }

    #[test]
    fn test_undo_moves_action_to_future() {
        let mut manager = ActionManager::new();
        let action = Box::new(MockAction::new());

        manager.execute(action);

        assert_eq!(manager.history.len(), 1);
        assert_eq!(manager.future.len(), 0);

        manager.undo();

        assert_eq!(manager.history.len(), 0);
        assert_eq!(manager.future.len(), 1);
    }

    #[test]
    fn test_redo_moves_action_back_to_history() {
        let mut manager = ActionManager::new();
        let action = Box::new(MockAction::new());

        manager.execute(action);
        manager.undo();

        assert_eq!(manager.history.len(), 0);
        assert_eq!(manager.future.len(), 1);

        manager.redo();

        assert_eq!(manager.history.len(), 1);
        assert_eq!(manager.future.len(), 0);
    }

    #[test]
    fn test_execute_calls_action_execute() {
        let mut manager = ActionManager::new();
        let action = Box::new(MockAction::new());
        let execute_flag = action.execute_called();

        manager.execute(action);

        // Check if execute was called
        let was_executed = *execute_flag.lock().unwrap();
        assert!(was_executed);
    }

    #[test]
    fn test_undo_calls_action_undo() {
        let mut manager = ActionManager::new();
        let action = Box::new(MockAction::new());
        let undo_flag = action.undo_called();

        manager.execute(action);
        manager.undo();

        // Check if undo was called
        let was_undone = *undo_flag.lock().unwrap();
        assert!(was_undone);
    }

    #[test]
    fn test_redo_calls_action_execute_again() {
        let mut manager = ActionManager::new();
        let action = Box::new(MockAction::new());
        let execute_flag = action.execute_called();

        manager.execute(action);
        manager.undo();
        manager.redo();

        // Check if execute was called again
        let was_executed = *execute_flag.lock().unwrap();
        assert!(was_executed);
    }

    #[test]
    fn test_multiple_actions() {
        let mut manager = ActionManager::new();

        let action1 = Box::new(MockAction::new());
        let action2 = Box::new(MockAction::new());

        let execute_flag1 = action1.execute_called();
        let undo_flag1 = action1.undo_called();

        let execute_flag2 = action2.execute_called();
        let undo_flag2 = action2.undo_called();

        manager.execute(action1);
        manager.execute(action2);

        assert_eq!(manager.history.len(), 2);
        assert_eq!(manager.future.len(), 0);

        manager.undo();
        assert_eq!(manager.history.len(), 1);
        assert_eq!(manager.future.len(), 1);

        manager.undo();
        assert_eq!(manager.history.len(), 0);
        assert_eq!(manager.future.len(), 2);

        // Check if undo was called on both actions
        let was_undone1 = *undo_flag1.lock().unwrap();
        let was_undone2 = *undo_flag2.lock().unwrap();
        assert!(was_undone1);
        assert!(was_undone2);

        manager.redo();
        assert_eq!(manager.history.len(), 1);
        assert_eq!(manager.future.len(), 1);

        manager.redo();
        assert_eq!(manager.history.len(), 2);
        assert_eq!(manager.future.len(), 0);

        // Check if execute was called again on both actions
        let was_executed1 = *execute_flag1.lock().unwrap();
        let was_executed2 = *execute_flag2.lock().unwrap();
        assert!(was_executed1);
        assert!(was_executed2);
    }
}
