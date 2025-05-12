# Experiment: ceres_selection_6

COMMIT: 3b6e2fd70ae67ac6a51f22f58a638bd042009c81

## Overview

The goal is to rerun `2025-05-02_09-57-15_ceres_selection_5` with larger search parameters to get reliable quality estimation.

## Results

Seems that patterns patterns are findable and tuning works (generally)

Corner count patterns
![](image.png)

In uncertain areas it is more difficult to catch the pattern:
![](image-1.png)

Surface construction is needed to find the best parameter using 3D plit

---

Comment from commit: 13389e5a6c1ad7e967cfdc2842871d55fe5c52a9

Surface visualization is helpful to find area with lower metric values. Global minimum however is not explicit as there are several local minimums.  

Visualization of uncertain area proportion metric:
![](image-2.png)
