use cxx;

#[cxx::bridge(namespace = "pinocchio")]
pub mod ffi_reference_frame {
    #[repr(u32)]
    pub enum CxxReferenceFrame {
        Local,
        World,
        LocalWorldAligned,
    }
}

pub enum ReferenceFrame {
    Local,
    World,
    LocalWorldAligned,
}

impl ReferenceFrame {
    fn from_cxx(cxx_reference_frame: &CxxReferenceFrame) -> ReferenceFrame {
        match cxx_reference_frame {
            CxxReferenceFrame::Local => ReferenceFrame::Local,
            CxxReferenceFrame::World => ReferenceFrame::World,
            CxxReferenceFrame::LocalWorldAligned => ReferenceFrame::LocalWorldAligned,
        }
    }

    fn to_cxx(&self) -> CxxReferenceFrame {
        match self {
            ReferenceFrame::Local => CxxReferenceFrame::Local,
            ReferenceFrame::World => CxxReferenceFrame::World,
            ReferenceFrame::LocalWorldAligned => CxxReferenceFrame::LocalWorldAligned,
        }
    }
}