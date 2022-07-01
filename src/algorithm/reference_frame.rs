pub enum ReferenceFrame {
    Local,
    World,
    LocalWorldAligned,
}

impl ReferenceFrame {
    pub fn to_u32(&self) -> u32 {
        match self {
            ReferenceFrame::Local => 0,
            ReferenceFrame::World => 1,
            ReferenceFrame::LocalWorldAligned => 2,
        }
    }
}