-- VOXEL_SIZE = 1e-1

include "transform.lua"

options = {
    tracking_frame = "base_link",
    pipeline = {
        {
            action = "min_max_range_filter",
            min_range = 1.0,  -- Minimum distance from the sensor
            max_range = 60.0, -- Maximum distance from the sensor
        },
        {
            action = "fixed_ratio_sampler",
            sampling_ratio = 0.1, -- Retain only 10% of the points
        },
        -- {
        --     action = "dump_num_points",
        -- },
        {
            action = "intensity_to_color",
            min_intensity = 0.0,
            max_intensity = 4096.0,
        },
        {
            action = "write_ply",
            filename = "map.ply",
        },
    },
}

return options

