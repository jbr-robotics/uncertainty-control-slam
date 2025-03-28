import os
import numpy as np

from cartographer_tuner.metrics.calculators.pgm.occupied_proportion_calculator import OccupiedProportionCalculator

def test_semi_occupied_proportion():
    """Test that semi_occupied.pgm has an occupied proportion between 40% and 60%."""

    pgm_path = os.path.join(os.path.dirname(__file__), "../../../data/semi_occupied.pgm")
    calculator = OccupiedProportionCalculator(map_path=pgm_path)
    
    results = calculator.calculate()
    proportion = results[OccupiedProportionCalculator.OCCUPIED_PROPORTION].value
    
    assert np.isclose(proportion, 0.4843, atol=0.01), f"Expected proportion to be close to 0.4843, got {proportion * 100:.2f}%"
