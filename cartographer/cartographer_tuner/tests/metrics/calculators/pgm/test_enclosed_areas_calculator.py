import os
import numpy as np

from cartographer_tuner.metrics.calculators.pgm.enclosed_areas_calculator import EnclosedAreasCalculator

def test_triangle_square_circle_enclosed_areas():
    """Test that triangle_square_circle.pgm has exactly 3 enclosed areas."""
    
    pgm_path = os.path.join(os.path.dirname(__file__), "../../../data/triangle_square_circle.pgm")
    calculator = EnclosedAreasCalculator(map_path=pgm_path)
    
    results = calculator.calculate()
    enclosed_areas_count = results[EnclosedAreasCalculator.ENLOSED_AREAS].value
    
    assert enclosed_areas_count == 3, f"Expected exactly 3 enclosed areas, got {enclosed_areas_count}"

def test_semi_occupied_enclosed_areas():
    """Test that semi_occupied.pgm has no enclosed areas."""
    
    pgm_path = os.path.join(os.path.dirname(__file__), "../../../data/semi_occupied.pgm")
    calculator = EnclosedAreasCalculator(map_path=pgm_path)
    
    results = calculator.calculate()
    enclosed_areas_count = results[EnclosedAreasCalculator.ENLOSED_AREAS].value
    
    assert enclosed_areas_count == 0, f"Expected exactly 0 enclosed area, got {enclosed_areas_count}"

def test_sample_map_enclosed_areas():
    """Test that sample_map.pgm has at most 5 enclosed areas."""
    
    pgm_path = os.path.join(os.path.dirname(__file__), "../../../data/sample_map.pgm")
    calculator = EnclosedAreasCalculator(map_path=pgm_path)
    
    results = calculator.calculate()
    enclosed_areas_count = results[EnclosedAreasCalculator.ENLOSED_AREAS].value
    
    assert enclosed_areas_count >= 5, f"Expected at least 5 enclosed areas, got {enclosed_areas_count}"
