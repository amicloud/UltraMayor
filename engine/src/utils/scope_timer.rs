pub struct ScopeTimer<'a> {
    name: &'a str,
    start_time: std::time::Instant,
}

impl<'a> ScopeTimer<'a> {
    pub fn new(name: &'a str) -> Self {
        Self {
            name,
            start_time: std::time::Instant::now(),
        }
    }
}

impl Drop for ScopeTimer<'_> {
    fn drop(&mut self) {
        let elapsed = self.start_time.elapsed();
        log::trace!("{} took {:.2?}", self.name, elapsed);
    }
}
