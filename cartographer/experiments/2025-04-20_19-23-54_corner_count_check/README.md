# Experiment: corner_count_check

COMMIT: eabc337722630b891d73cbe570e844e5efc5c6f6

## Overview

Goal: check if default corner count detector from commit `e649d3bc` works bettern than modified one at `bf5b9b77f26d`.

Updated calculator detects many extra corners.
![](../2025-04-19_14-10-01_num_accumulated_range_data_selection_5/image-2.png)

Expected that old one detects ~14-20 cornerns.

## Results

Experiment failed, too many extra corners detected. 
![](image.png)

Decided to drop discrete dots.

Moreover, it is noted that `corner count` is more reasonable if measured on layers other than `alpha`

![](image-1.png)

## Note

submap `pkl` is saved [here](./submap_3.pkl) 