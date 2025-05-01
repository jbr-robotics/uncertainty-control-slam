import numpy as np
import cv2
from typing import Dict, List, Optional, Union

from cartographer_tuner.metrics.calculators.pgm.base_pgm_metric_calculator import BasePgmMetricCalculator
from cartographer_tuner.metrics.metric import Metric
from cartographer_tuner.utils.visualization import show_image

__all__ = ["UnsureAreaProportionCalculator"]

UNKNOWN_VALUE = 127

class UnsureAreaProportionCalculator(BasePgmMetricCalculator):
    """
    Calculates the proportion of map cells that are dynamically classified as 'unsure'.

    Classification Strategy:
    1. Excludes cells with UNKNOWN_VALUE (127).
    2. Applies K-Means clustering (K=3) to the remaining ('known') cell intensity values.
    3. The cluster with the lowest centroid is considered 'occupied'.
    4. The cluster with the highest centroid is considered 'unoccupied'.
    5. The cluster with the middle centroid is considered 'unsure'.
    6. Handles edge cases where there are fewer than 3 unique known intensity values.

    The proportion is calculated as:
        unsure_cells / total_known_cells
    """
    UNSURE_AREA_PROPORTION = "unsure_area_proportion"
    METRIC_NAMES = [UNSURE_AREA_PROPORTION]

    # --- K-Means Parameters ---
    # Criteria for K-Means termination
    # (type, max_iter, epsilon)
    KMEANS_CRITERIA = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 1.0)
    # Number of times algorithm is executed using different initial labellings.
    KMEANS_ATTEMPTS = 10
    # How initial centers are taken.
    KMEANS_FLAGS = cv2.KMEANS_PP_CENTERS
    # Number of clusters
    KMEANS_K = 3

    # Threshold to consider a single value cluster as 'occupied'
    SINGLE_VALUE_OCCUPIED_THRESH = 64
    SINGLE_VALUE_UNOCCUPIED_THRESH = 192


    def __init__(
        self,
        map_data: Union[str, np.ndarray],
        debug: bool = False,
        **kwargs,
    ):
        """
        Initializes the calculator.

        Args:
            map_data: Path to the PGM map file or a NumPy array containing map data.
                      Intensity values are clustered dynamically. Value 127 is ignored.
            debug: If True, enables debug logging and visualizations.
            **kwargs: Additional keyword arguments passed to the base class.
        """
        super().__init__(map_data)
        self.debug = debug
        assert self.map_data.dtype == np.uint8, "Map data must be uint8"

    def _classify_cells(self) -> tuple[Optional[np.ndarray], Optional[np.ndarray], int]:
        """
        Internal helper to perform classification.

        Returns:
            tuple: (labels, centers, unsure_label_index) if K-Means runs,
                   (None, None, determined_unsure_label (-1 if none)) for edge cases,
                   or raises error.
            Returns None for labels/centers if classification is trivial (edge cases).
            unsure_label_index: The label index representing the unsure class, or -1 if no unsure cells
                                are identified in edge cases.
        """
        # 1. Filter out unknown cells
        known_mask = self.map_data != UNKNOWN_VALUE
        known_values = self.map_data[known_mask]
        self.total_known_cells = len(known_values)

        if self.total_known_cells == 0:
            if self.debug: print("Debug: No known cells found.")
            return None, None, -1

        # 2. Analyze unique values for edge cases
        unique_known_values = np.unique(known_values)
        num_unique_values = len(unique_known_values)


        # 3. Handle Edge Cases (Fewer than K unique values)
        if num_unique_values < self.KMEANS_K:
            if num_unique_values == 1:
                # Only one intensity value present (excluding unknown)
                value = unique_known_values[0]
                if value < self.SINGLE_VALUE_OCCUPIED_THRESH or value > self.SINGLE_VALUE_UNOCCUPIED_THRESH:
                    # Consider it occupied/unoccupied
                    return None, None, -1 # No unsure
                else:
                    # Consider it unsure
                     return None, None, 0 # All unsure, assign label 0
            elif num_unique_values == 2:
                 # Assume the two values represent occupied and unoccupied
                 return None, None, -1 # No unsure
            else: 
                assert False, f"Unexpected edge case with {num_unique_values} unique values."

        # 4. Perform K-Means Clustering (Standard Case)
        data_for_kmeans = known_values.reshape(-1, 1).astype(np.float32)

        try:
            compactness, labels_flat, centers = cv2.kmeans(data      = data_for_kmeans,
                                                        K         = self.KMEANS_K,
                                                        bestLabels= None,
                                                        criteria  = self.KMEANS_CRITERIA,
                                                        attempts  = self.KMEANS_ATTEMPTS,
                                                        flags     = self.KMEANS_FLAGS)
        except cv2.error as e:
            print(f"Error during cv2.kmeans: {e}")
            assert False, f"K-Means clustering failed. Cannot calculate unsure proportion."

        # 5. Identify the 'unsure' cluster (middle centroid)
        center_values = centers.flatten()
        sorted_center_indices = np.argsort(center_values)
        unsure_label_index = sorted_center_indices[1] # Index 1 is middle for K=3

        full_labels = np.full(self.map_data.shape, -1, dtype=np.int32)
        known_mask = self.map_data != UNKNOWN_VALUE
        full_labels[known_mask] = labels_flat.flatten()


        return full_labels, centers, unsure_label_index


    def calculate(self, metrics: Optional[List[str]] = None) -> Dict[str, Metric]:
        """
        Calculates the dynamic unsure area proportion metric.

        Args:
            metrics: A list of metric names to calculate. If None, calculates all
                     metrics defined in METRIC_NAMES.

        Returns:
            A dictionary mapping metric names to Metric objects.
        """
        metrics = self._process_metric_names(metrics) # Use base class helper
        results = {}

        if self.UNSURE_AREA_PROPORTION in metrics:
            unsure_cells_count = 0
            proportion = 0.0

            try:
                labels, centers, unsure_label_index = self._classify_cells()

                assert self.total_known_cells > 0, "No known cells found."
                if labels is not None and unsure_label_index != -1:
                    # K-Means case: Count cells with the unsure label
                    unsure_cells_count = np.sum(labels == unsure_label_index)
                elif labels is None and unsure_label_index == 0:
                        # Edge case: Single value classified as unsure
                        unsure_cells_count = self.total_known_cells

                # Calculate proportion
                proportion = unsure_cells_count / self.total_known_cells
            except Exception as e:
                assert False, f"Error calculating {self.UNSURE_AREA_PROPORTION}: {e}"

            results[self.UNSURE_AREA_PROPORTION] = Metric(
                name=self.UNSURE_AREA_PROPORTION,
                value=float(proportion),
            )

        return results

    def debug_image(self):
        """
        Generates a debug image highlighting dynamically classified cells.
        - Red: Occupied
        - Yellow: Unsure
        - Green: Unoccupied
        - Original Gray: Unknown (127) or Error

        Returns:
            A NumPy array representing the debug image (BGR format).
        """
        try:
            labels, centers, unsure_label_index = self._classify_cells()

            assert len(self.map_data.shape) == 2, "Map data must be grayscale"
            output_img = cv2.cvtColor(self.map_data, cv2.COLOR_GRAY2RGB)

            occupied_color = [0, 0, 0]   # Black
            unsure_color = [255, 0, 0] # Red
            unoccupied_color = [255, 255, 255] # White
            unknown_color = [127, 127, 127] # Gray for original unknown

            if labels is not None and centers is not None and unsure_label_index != -1 :
                # K-Means was successful
                center_values = centers.flatten()
                sorted_center_indices = np.argsort(center_values)
                occupied_label_index = sorted_center_indices[0]
                # unsure_label_index = sorted_center_indices[1] # Already have this
                unoccupied_label_index = sorted_center_indices[2]

                # Apply colors based on labels
                output_img[labels == occupied_label_index] = occupied_color
                output_img[labels == unsure_label_index] = unsure_color
                output_img[labels == unoccupied_label_index] = unoccupied_color
                # Keep original unknown values gray - reset them
                output_img[self.map_data == UNKNOWN_VALUE] = unknown_color


            elif labels is None: # Handle edge cases visualized
                 known_mask = self.map_data != UNKNOWN_VALUE
                 if unsure_label_index == 0: # All known are unsure
                     output_img[known_mask] = unsure_color
                 elif unsure_label_index == -1: # No unsure (e.g. 1 or 2 unique values)
                     unique_known = np.unique(self.map_data[known_mask])
                     if len(unique_known) == 1:
                         val = unique_known[0]
                         if val < self.SINGLE_VALUE_OCCUPIED_THRESH:
                             output_img[known_mask] = occupied_color
                         elif val > self.SINGLE_VALUE_UNOCCUPIED_THRESH:
                             output_img[known_mask] = unoccupied_color
                         # else case where it would have been unsure is handled above
                     elif len(unique_known) == 2:
                          low_val, high_val = np.sort(unique_known)
                          output_img[(self.map_data == low_val) & known_mask] = occupied_color
                          output_img[(self.map_data == high_val) & known_mask] = unoccupied_color

                 # Ensure original unknown cells are visually distinct if desired
                 output_img[self.map_data == UNKNOWN_VALUE] = unknown_color

            return output_img

        except Exception as e:
            assert False, f"Error generating debug image: {e}"