VOXEL_SIZE = 5e-2

include "transform.lua"

options = {
    tracking_frame = "base_link",
    pipeline = {
        {
        action = "write_ply",
        filename = "trajectory.ply",
        },
    },
}
  
return options
