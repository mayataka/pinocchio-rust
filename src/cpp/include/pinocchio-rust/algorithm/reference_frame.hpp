#pragma once

#include "pinocchio/multibody/fwd.hpp"

namespace pinocchio {

inline ReferenceFrame ReferenceFrameFromUint32(const std::uint32_t rf) {
  switch (rf)
  {
  case 0:
    return ReferenceFrame::LOCAL;
  case 1:
    return ReferenceFrame::WORLD;
  default:
    return ReferenceFrame::LOCAL_WORLD_ALIGNED;
  }
}

}