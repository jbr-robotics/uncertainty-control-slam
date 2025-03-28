import lupa
from lupa import LuaRuntime
from pathlib import Path
import subprocess
import shutil
from typing import Dict, Any, Union, Optional, List

from cartographer_tuner.core.exceptions import (
    ConfigLoadException,
    ConfigParseException,
    InvalidParameterException,
    ConfigFileException,
)

from cartographer_tuner.exceptions import CartographerDependencyException

class CartographerConfigManager:
    """
    Manages Cartographer SLAM configuration files.
    
    This class provides functionality to load, manipulate, and export Cartographer
    configuration files, which are stored in Lua format.
    
    Requires the external tool 'cartographer_print_configuration' to be installed.
    """
    
    def __init__(self):
        """Initialize a new CartographerConfigManager instance."""
        self.lua = LuaRuntime(unpack_returned_tuples=True)
        self._config: Dict[str, Any] = {}
        self._verify_dependencies()
    
    def _verify_dependencies(self) -> None:
        """
        Verify that all required external tools are available.
        
        Raises:
            CartographerDependencyException: If a required dependency is missing
        """
        if not shutil.which("cartographer_print_configuration"):
            raise CartographerDependencyException(
                "Required tool 'cartographer_print_configuration' is missing. "
                "Please install it via ROS packages before using this package."
            )
    
    def load(self, config_path: Union[str, Path], config_dir: Optional[str] = None) -> Dict[str, Any]:
        """
        Load a Cartographer configuration file.
        
        Args:
            config_path: Path to the configuration file or basename if config_dir is provided
            config_dir: Directory containing configuration files (optional)
            
        Returns:
            Dict containing the parsed configuration
            
        Raises:
            ConfigLoadException: If the configuration can't be loaded
            ConfigParseException: If the configuration can't be parsed
        """
        config_path = Path(config_path)
        
        if config_dir is None:
            config_dir = str(config_path.parent)
            config_basename = config_path.name
        else:
            config_basename = str(config_path)
            
        try:
            lua_code = self._run_cartographer_config_tool(config_dir, config_basename)
            
            try:
                parsed_result = self.lua.execute(lua_code)
                raw_config = self._lua_table_to_python(parsed_result)
            except Exception as e:
                raise ConfigParseException(f"Failed to parse configuration: {str(e)}")
            
            if isinstance(raw_config, dict) and "options" in raw_config:
                self._config = raw_config["options"]
            else:
                self._config = raw_config
                
            return self._config
            
        except (ConfigParseException, ConfigLoadException):
            raise
        except Exception as e:
            raise ConfigLoadException(f"Failed to load configuration: {str(e)}")
    
    def _run_cartographer_config_tool(self, config_dir: str, config_basename: str) -> str:
        """
        Run cartographer_print_configuration to get the full configuration.
        
        Args:
            config_dir: Directory containing configuration files
            config_basename: Configuration file basename
            
        Returns:
            String containing Lua code with the full configuration
            
        Raises:
            ConfigLoadException: If the tool fails
        """
        cmd = [
            "cartographer_print_configuration",
            f"-configuration_directories={config_dir}",
            f"-configuration_basename={config_basename}"
        ]
        
        try:
            result = subprocess.run(
                cmd, 
                capture_output=True, 
                text=True, 
                check=True
            )
            return result.stdout
        except subprocess.CalledProcessException as e:
            raise ConfigLoadException(f"Failed to run cartographer_print_configuration: {e.stderr}")
    
    def _lua_table_to_python(self, obj) -> Union[Dict, List, Any]:
        """
        Convert a Lua table to a Python dict or list.
        
        Args:
            obj: Lua object to convert
            
        Returns:
            Converted Python object (dict, list, or primitive type)
        """
        if lupa.lua_type(obj) == 'table':
            keys = list(obj.keys())
            if keys and keys == list(range(1, len(keys) + 1)):
                return [self._lua_table_to_python(obj[key]) for key in keys]
            else:
                return {key: self._lua_table_to_python(obj[key]) for key in obj}
        return obj
    
    def to_dict(self) -> Dict[str, Any]:
        """
        Get the complete configuration as a dictionary.
        
        Returns:
            Complete configuration dictionary
        """
        return self._config
    
    def get(self, param_path: str) -> Any:
        """
        Get a parameter from the loaded configuration using dot notation.
        
        Args:
            param_path: Path to the parameter using dot notation (e.g., "trajectory_builder.mapping_2d.submaps.num_range_data")
            
        Returns:
            Parameter value
            
        Raises:
            InvalidParameterException: If the parameter or any part of its path doesn't exist
        """
        try:
            return self[param_path]
        except KeyError as e:
            raise InvalidParameterException(str(e))
    
    def set(self, param_path: str, value: Any) -> None:
        """
        Set a parameter in the loaded configuration using dot notation.
        
        Args:
            param_path: Path to the parameter using dot notation
            value: Value to set
            
        Raises:
            InvalidParameterException: If the parameter path can't be created
        """
        try:
            self[param_path] = value
        except KeyError as e:
            raise InvalidParameterException(str(e))
    
    def _dict_to_lua_string(self, d: Dict[str, Any], indent: int = 0) -> str:
        """
        Convert a Python dict to a Lua table string representation.
        
        Args:
            d: Dictionary to convert
            indent: Current indentation level
            
        Returns:
            String representation of the dict as a Lua table
        """
        lines = []
        spaces = " " * (indent * 2)
        
        for k, v in d.items():
            if isinstance(v, dict):
                lines.append(f"{spaces}{k} = {{")
                lines.append(self._dict_to_lua_string(v, indent + 1))
                lines.append(f"{spaces}}},")
            elif isinstance(v, list):
                lines.append(f"{spaces}{k} = {{")
                list_items = [self._value_to_lua_string(item, indent + 1) for item in v]
                lines.append(",\n".join(list_items))
                lines.append(f"{spaces}}},")
            else:
                lines.append(f"{spaces}{k} = {self._value_to_lua_string(v)},")
                
        return "\n".join(lines)
    
    def _value_to_lua_string(self, v: Any, indent: int = 0) -> str:
        """
        Convert a Python value to its Lua string representation.
        
        Args:
            v: Value to convert
            indent: Current indentation level
            
        Returns:
            String representation of the value in Lua
        """
        spaces = " " * (indent * 2)
        
        if isinstance(v, dict):
            inner_content = self._dict_to_lua_string(v, indent + 1)
            return f"{{\n{inner_content}\n{spaces}}}"
        elif isinstance(v, list):
            list_items = [self._value_to_lua_string(item, indent + 1) for item in v]
            return f"{{\n{spaces}  " + f",\n{spaces}  ".join(list_items) + f"\n{spaces}}}"
        elif isinstance(v, bool):
            return str(v).lower()
        elif isinstance(v, (int, float)):
            return str(v)
        elif isinstance(v, str):
            return f'"{v}"'
        else:
            return str(v)
    
    def save_to_file(self, file_path: Union[str, Path]) -> None:
        """
        Save the configuration to a Lua file.
        
        Args:
            file_path: Path where to save the configuration
            
        Raises:
            ConfigFileException: If the file cannot be written
        """
        try:
            with open(file_path, 'w') as f:
                f.write(str(self))
        except (IOError, OSError) as e:
            raise ConfigFileException(f"Failed to save configuration to {file_path}: {str(e)}")
    
    def __str__(self) -> str:
        """
        Convert the configuration to a Lua string representation.
        
        This format matches what Cartographer expects in its configuration files.
        
        Returns:
            String containing Lua code with the configuration
        """
        return f"options = {{\n{self._dict_to_lua_string(self._config, 1)}\n}}\n\nreturn options"
    
    
    def __getitem__(self, key: str) -> Any:
        """
        Access configuration parameters using dot notation.
        
        Example:
            config["trajectory_builder.mapping_2d.submaps.num_range_data"]
            
        Args:
            key: Parameter path using dot notation
            
        Returns:
            Parameter value
            
        Raises:
            KeyError: If the parameter doesn't exist
        """
        if not self._config:
            raise KeyError("No configuration loaded")
            
        current = self._config
        if not key:
            return current
            
        parts = key.split('.')
        
        for i, part in enumerate(parts):
            if not isinstance(current, dict):
                raise KeyError(f"Cannot access '{part}' in '{'.'.join(parts[:i])}': not a dictionary")
            
            if part not in current:
                raise KeyError(f"Key not found: {key}")
                
            current = current[part]
            
        return current
    
    def __setitem__(self, key: str, value: Any) -> None:
        """
        Set configuration parameters using dot notation.
        
        Example:
            config["trajectory_builder.mapping_2d.submaps.num_range_data"] = 42
            
        Args:
            key: Parameter path using dot notation
            value: Value to set
            
        Raises:
            KeyError: If the parameter path can't be created
        """
        if not self._config:
            self._config = {}
            
        current = self._config
        parts = key.split('.')
        
        for part in parts[:-1]:
            if part not in current:
                current[part] = {}
            elif not isinstance(current[part], dict):
                raise KeyError(f"Cannot set key '{key}': parent is not a dictionary")
                
            current = current[part]
            
        current[parts[-1]] = value
