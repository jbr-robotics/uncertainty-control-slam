# Experiment: num_range_data_selection_2

COMMIT: 328958d0e7af636e66f114c71e631b135e4aac58

## Overview

Goal: rerun `2025-04-17_21-45-42_num_range_data_selection` but with properly selected `num_accumulated_range_data` (see experiment `2025-04-19_14-10-01_num_accumulated_range_data_selection_5`) to find best `num_range_data`. 

## Results

Experiment interrupted. The problem is that scans are not properly matched. Below there is an example for one map:

![](image.png)
![](image-1.png)
![](image-2.png)
![](image-3.png)
![](image-4.png)
![](image-5.png)
![](image-6.png)
![](image-7.png)

It leads to very poor map at higher `num_range_data` values

![](image-8.png)

As a result, the data is very noise and not representative

![](image-11.png)
![](image-12.png)

It is suggested to tune ceres parameters first.