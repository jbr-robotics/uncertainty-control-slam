import os

from cartographer_tuner.metrics.calculators.pgm.corner_count_calculator import CornerCountCalculator

def test_triangle_square_circle_corner_count():
    """Test that triangle_square_circle.pgm has at least 7 corners."""
    
    pgm_path = os.path.join(os.path.dirname(__file__), "../../../data/triangle_square_circle.pgm")
    calculator = CornerCountCalculator(map_path=pgm_path)
    
    results = calculator.calculate()
    corner_count = results["corner_count"].value
    
    assert 7 <= corner_count, f"Expected at least 7 corners (3 from the triangle, 4 from the square, and >= 0 from the circle), got {corner_count}"
    assert corner_count <= 20, f"Expected at most 20 corners (not too much corners from rectangle), got {corner_count}"

def test_sample_map_corner_count():
    """Test that sample_map.pgm has adequate corner count."""
    
    pgm_path = os.path.join(os.path.dirname(__file__), "../../../data/sample_map.pgm")
    calculator = CornerCountCalculator(map_path=pgm_path)
    
    results = calculator.calculate()
    corner_count = results["corner_count"].value
    
    assert 50 <= corner_count <= 300, f"Expected between 50 and 300 corners, got {corner_count}"
