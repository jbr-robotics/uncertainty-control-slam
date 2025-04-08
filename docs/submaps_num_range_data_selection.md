# Num Range Data Selection

Current document has descption of the approach on how to select optimal `TRAJECTORY_BUILDER_2D.submaps.num_range_data` value for specified platform and environment. 

## The Role of `num_range_data`

According to the [documentation](https://google-cartographer-ros.readthedocs.io/en/latest/algo_walkthrough.html#local-slam-1):
> A submap is considered as complete when the local SLAM has received a given amount of range data. Local SLAM drifts over time, global 
SLAM is used to fix this drift. Submaps must be small enough so that the drift inside them is below the resolution, so that they are 
locally correct. On the other hand, they should be large enough to be distinct for loop closure to work properly.

Therefore, our selection method aims to identify the largest possible value of `num_range_data` that ensures submaps do not drift.

## Hypothesis

With an appropriate metric for submap quality, we can optimize the mean value of this metric across submaps. However, due to significant variations between submaps, it's important to:
1. Classify submaps into categories
2. Select a value that performs well across multiple categories
3. Recognize that some categories may show large variations regardless of the `num_range_data` value used

We propose that classification using buckets based on the `corner_count` metric, combined with a confidence metric that evaluates map accuracy, would provide an effective approach.

## Experiment Design

1. Test `num_range_data` values within a range $[L, R]$ at intervals of step size $S$
2. Run Cartographer with each `num_range_data` value
3. Extract all submaps and their versions for statistical analysis
4. Construct a 2-dimensional histogram with:
   - X-axis: `num_range_data` values
   - Y-axis: `corner_count` buckets
   - Z-axis: mean value of map quality metric
   - (Also calculate variation for each (x, y) bucket)
5. For each `num_range_data` value, exclude categories with large variations and evaluate overall performance "looking at the histogram"

### Concerns Regarding Experiment Design

- While using the complete history of each submap provides more data, older versions of submaps are not used in constructing the final global map. Including them might introduce noise into the analysis for two key reasons: 
    * Intermediate submap states do not contribute to the final map quality, yet our ultimate goal in tuning Cartographer is to produce an optimal overall map.
    * Early stages of submaps are nearly identical across all `num_range_data` values, which means metrics would be calculated from largely redundant data, reducing the statistical relevance of our comparisons.
- Comparing small and large values of `num_range_data` is challenging because `corner_count` distributions will differ significantly between them.
- Submaps with a larger number of versions will disproportionately influence the analysis, creating statistical bias toward their characteristics

## Alternative Experiment Design

1. Test `num_range_data` values within a range $[L, R]$ at intervals of step size $S$
2. For each `num_range_data` value, sample $B$ different starting points $s_1, s_2, ..., s_B$
3. Run Cartographer $B$ times, with the $i$-th iteration starting from position $s_i$
4. Extract only the first completed submap from each run for statistical analysis
5. Construct a 2-dimensional histogram as described in the original experiment design
6. Evaluate each `num_range_data` value using only the `corner_count` buckets that exhibit "small" variation

### Advantages of the Alternative Approach

1. Ensures uniform sample size across different `num_range_data` values, creating a more balanced comparison
2. Produces less noisy analysis by including only final (completed) submaps rather than intermediate versions
3. When analyzing a fixed `num_range_data` value, variations in `corner_count` result from sampling different recording times and locations, rather than from comparing different versions of the same submap. 

### Concerns Regarding Alternative Approach

- Comparing large and small values of `num_range_data` remains challenging because the `corner_count` metric will naturally differ between them, making direct comparisons potentially misleading.

## Notes and Ideas for Further Enhancement

- Large values of `num_range_data` lead to drift, while small values result in poor submap detail. Ideally, we need two metrics:
  1. A metric measuring drift
  2. A metric measuring detail quality
- With these metrics, we could select the optimal `num_range_data` by finding the balance point between drift and detail
- Detail quality could potentially be measured as the mean value across intensity map regions where certainty exceeds zero