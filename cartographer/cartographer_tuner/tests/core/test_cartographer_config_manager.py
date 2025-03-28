"""Tests for the CartographerConfigManager class."""

import pytest
from pathlib import Path
import subprocess
from unittest.mock import patch, MagicMock, mock_open

from cartographer_tuner.core.cartographer_config_manager import CartographerConfigManager
from cartographer_tuner.core.exceptions import (
    ConfigLoadError,
    ConfigParseError,
    InvalidParameterError,
    ConfigFileError,
)
from cartographer_tuner.exceptions import CartographerDependencyError

# Test data paths
DATA_DIR = Path(__file__).parent.parent / "data"
SAMPLE_CONFIG_PATH = DATA_DIR / "sample_config.lua"

@pytest.fixture
def sample_lua():
    """Load sample Lua configuration."""
    with open(SAMPLE_CONFIG_PATH, 'r') as f:
        return f.read()

@pytest.fixture
def mock_cartographer_dependency():
    """Mock cartographer_print_configuration dependency."""
    with patch('shutil.which') as mock_which:
        mock_which.return_value = '/path/to/cartographer_print_configuration'
        yield mock_which

@pytest.fixture
def config_manager(mock_cartographer_dependency):
    """Create a CartographerConfigManager instance."""
    return CartographerConfigManager()

def test_init_dependency_check_success(mock_cartographer_dependency):
    """Test successful dependency check during initialization."""
    manager = CartographerConfigManager()
    assert manager is not None

def test_init_dependency_check_failure():
    """Test failed dependency check during initialization."""
    with patch('shutil.which') as mock_which:
        mock_which.return_value = None
        with pytest.raises(CartographerDependencyError):
            CartographerConfigManager()

def test_load_config_success(config_manager, sample_lua):
    """Test successful configuration loading."""
    with patch('subprocess.run') as mock_run:
        mock_process = MagicMock()
        mock_process.stdout = sample_lua
        mock_process.returncode = 0
        mock_run.return_value = mock_process
        
        config = config_manager.load(SAMPLE_CONFIG_PATH)
        
        # Check configuration structure
        assert 'map_builder' in config
        assert 'trajectory_builder' in config
        assert config['map_builder']['use_trajectory_builder_2d'] is True
        assert config['trajectory_builder']['trajectory_builder_2d']['max_range'] == 30.0
        
        # Check subprocess call
        mock_run.assert_called_once()
        args, kwargs = mock_run.call_args
        assert 'cartographer_print_configuration' in args[0][0]

def test_load_config_with_config_dir(config_manager, sample_lua):
    """Test loading configuration with explicit config_dir."""
    with patch('subprocess.run') as mock_run:
        mock_process = MagicMock()
        mock_process.stdout = sample_lua
        mock_process.returncode = 0
        mock_run.return_value = mock_process
        
        config_dir = "/path/to/config"
        config_manager.load("base_config.lua", config_dir=config_dir)
        
        # Check subprocess call with config_dir
        mock_run.assert_called_once()
        args, kwargs = mock_run.call_args
        assert f"-configuration_directories={config_dir}" in args[0][1]
        assert "-configuration_basename=base_config.lua" in args[0][2]

def test_load_config_subprocess_error(config_manager):
    """Test configuration loading when subprocess fails."""
    with patch('subprocess.run') as mock_run:
        mock_run.side_effect = subprocess.CalledProcessError(
            returncode=1,
            cmd='cartographer_print_configuration',
            stderr='Some error output'
        )
        
        with pytest.raises(ConfigLoadError):
            config_manager.load(SAMPLE_CONFIG_PATH)

def test_load_config_parse_error(config_manager):
    """Test configuration loading when parsing fails."""
    with patch('subprocess.run') as mock_run:
        mock_process = MagicMock()
        mock_process.stdout = "This is not valid Lua code"
        mock_process.returncode = 0
        mock_run.return_value = mock_process
        
        with pytest.raises(ConfigParseError):
            config_manager.load(SAMPLE_CONFIG_PATH)

def test_create_config_from_scratch(config_manager):
    """Test creating a configuration from scratch."""
    # Create a configuration from scratch
    config_manager.set('map_builder.use_trajectory_builder_2d', True)
    config_manager.set('map_builder.use_trajectory_builder_3d', False)
    config_manager.set('map_builder.num_background_threads', 4)
    config_manager.set('trajectory_builder.trajectory_builder_2d.use_imu_data', True)
    config_manager.set('trajectory_builder.trajectory_builder_2d.max_range', 30.0)
    
    # Verify the configuration
    assert config_manager.get('map_builder.use_trajectory_builder_2d') is True
    assert config_manager.get('map_builder.use_trajectory_builder_3d') is False
    assert config_manager.get('map_builder.num_background_threads') == 4
    assert config_manager.get('trajectory_builder.trajectory_builder_2d.use_imu_data') is True
    assert config_manager.get('trajectory_builder.trajectory_builder_2d.max_range') == 30.0
    
    # Check the structure of the config
    config = config_manager.to_dict()
    assert 'map_builder' in config
    assert 'trajectory_builder' in config
    
    # Convert to Lua string and check format
    lua_string = str(config_manager)
    assert lua_string.startswith('options = {')
    assert 'map_builder' in lua_string
    assert 'trajectory_builder' in lua_string

def test_get_parameter(config_manager, sample_lua):
    """Test getting parameters using dot notation."""
    with patch('subprocess.run') as mock_run:
        mock_process = MagicMock()
        mock_process.stdout = sample_lua
        mock_process.returncode = 0
        mock_run.return_value = mock_process
        
        config_manager.load(SAMPLE_CONFIG_PATH)
        
        # Test getting parameters with various paths
        assert config_manager.get('map_builder.use_trajectory_builder_2d') is True
        assert config_manager.get('map_builder.num_background_threads') == 4.0
        assert config_manager.get('trajectory_builder.trajectory_builder_2d.max_range') == 30.0
        
        # Test getting deeply nested parameters
        assert config_manager.get('trajectory_builder.trajectory_builder_2d.submaps.grid_options_2d.resolution') == 0.05
        
        # Test get using dict-like syntax
        assert config_manager['map_builder.use_trajectory_builder_2d'] is True
        
        # Test getting non-existent parameter
        with pytest.raises(InvalidParameterError):
            config_manager.get('non_existent_key')
        
        # Test getting parameter with invalid path
        with pytest.raises(InvalidParameterError):
            config_manager.get('map_builder.use_trajectory_builder_2d.invalid')

def test_set_parameter(config_manager, sample_lua):
    """Test setting parameters using dot notation."""
    with patch('subprocess.run') as mock_run:
        mock_process = MagicMock()
        mock_process.stdout = sample_lua
        mock_process.returncode = 0
        mock_run.return_value = mock_process
        
        config_manager.load(SAMPLE_CONFIG_PATH)
        
        # Set parameters and verify they were set correctly
        config_manager.set('map_builder.use_trajectory_builder_2d', False)
        assert config_manager.get('map_builder.use_trajectory_builder_2d') is False
        
        config_manager.set('trajectory_builder.trajectory_builder_2d.max_range', 50.0)
        assert config_manager.get('trajectory_builder.trajectory_builder_2d.max_range') == 50.0
        
        # Set a new parameter
        config_manager.set('new_parameter', 'new_value')
        assert config_manager.get('new_parameter') == 'new_value'
        
        # Set a nested new parameter
        config_manager.set('map_builder.new_param', 'nested_value')
        assert config_manager.get('map_builder.new_param') == 'nested_value'
        
        # Set a deeply nested parameter
        config_manager.set('trajectory_builder.trajectory_builder_2d.submaps.grid_options_2d.resolution', 0.1)
        assert config_manager.get('trajectory_builder.trajectory_builder_2d.submaps.grid_options_2d.resolution') == 0.1
        
        # Test set using dict-like syntax
        config_manager['map_builder.use_trajectory_builder_3d'] = True
        assert config_manager.get('map_builder.use_trajectory_builder_3d') is True
        
        # Test setting parameter with invalid path
        with pytest.raises(InvalidParameterError):
            config_manager.set('map_builder.use_trajectory_builder_2d.invalid', 'value')

def test_to_lua_string(config_manager, sample_lua):
    """Test converting configuration to Lua string."""
    with patch('subprocess.run') as mock_run:
        mock_process = MagicMock()
        mock_process.stdout = sample_lua
        mock_process.returncode = 0
        mock_run.return_value = mock_process
        
        config_manager.load(SAMPLE_CONFIG_PATH)
        
        # Convert to Lua string
        lua_string = str(config_manager)
        
        # Check that the string has the correct format
        assert lua_string.startswith('options = {')
        assert lua_string.endswith('return options')
        
        # Check that key elements exist in the string
        assert 'map_builder' in lua_string
        assert 'use_trajectory_builder_2d' in lua_string
        assert 'trajectory_builder' in lua_string
        assert 'max_range' in lua_string

def test_save_to_file(config_manager, sample_lua):
    """Test saving configuration to a file."""
    with patch('subprocess.run') as mock_run:
        mock_process = MagicMock()
        mock_process.stdout = sample_lua
        mock_process.returncode = 0
        mock_run.return_value = mock_process
        
        config_manager.load(SAMPLE_CONFIG_PATH)
        
        # Test file path
        output_path = DATA_DIR / 'output_config.lua'
        
        # Mock the open function
        m = mock_open()
        with patch('builtins.open', m):
            config_manager.save_to_file(output_path)
        
        # Check that the file was opened and written to
        m.assert_called_with(output_path, 'w')
        m().write.assert_called_once()
        
        # Check that what was written is the string representation
        written_content = m().write.call_args[0][0]
        assert written_content == str(config_manager)

def test_save_to_file_error(config_manager, sample_lua):
    """Test error handling when saving fails."""
    with patch('subprocess.run') as mock_run:
        mock_process = MagicMock()
        mock_process.stdout = sample_lua
        mock_process.returncode = 0
        mock_run.return_value = mock_process
        
        config_manager.load(SAMPLE_CONFIG_PATH)
        
        # Test file path
        output_path = DATA_DIR / 'output_config.lua'
        
        # Mock the open function to raise an IOError
        m = mock_open()
        m.side_effect = IOError("Failed to write file")
        
        with patch('builtins.open', m):
            with pytest.raises(ConfigFileError):
                config_manager.save_to_file(output_path)

def test_deep_nested_parameters(config_manager):
    """Test handling deeply nested parameters."""
    # Set deeply nested parameters
    config_manager.set('a.b.c.d.e.f', 'nested_value')
    config_manager.set('a.b.c.d.e.g', 123)
    config_manager.set('a.b.c.d.e.h', True)
    
    # Check values
    assert config_manager.get('a.b.c.d.e.f') == 'nested_value'
    assert config_manager.get('a.b.c.d.e.g') == 123
    assert config_manager.get('a.b.c.d.e.h') is True
    
    # Check structure
    assert config_manager.get('a') is not None
    assert config_manager.get('a.b') is not None
    assert config_manager.get('a.b.c') is not None
    assert config_manager.get('a.b.c.d') is not None
    assert config_manager.get('a.b.c.d.e') is not None
    
    # Get intermediate structures
    d_dict = config_manager.get('a.b.c.d')
    assert isinstance(d_dict, dict)
    assert 'e' in d_dict
    
    # Check full path
    a_dict = config_manager.get('a')
    assert isinstance(a_dict, dict)
    assert 'b' in a_dict
    assert isinstance(a_dict['b'], dict)
    assert 'c' in a_dict['b'] 