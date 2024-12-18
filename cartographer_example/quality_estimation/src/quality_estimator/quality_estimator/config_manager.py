import lupa
from lupa import LuaRuntime
from pathlib import Path
import subprocess
from typing import Dict, Any, Union, Optional

class ConfigManager:
    """Class to manage Lua configuration files.
    
    This class provides functionality to:
    1. Load Lua configuration files using cartographer_print_configuration
    2. Store configurations in a Python dictionary structure
    3. Convert Python configurations back to Lua format
    """
    
    def __init__(self):
        """Initialize the config manager."""
        self.lua = LuaRuntime(unpack_returned_tuples=True)
        self.configs: Dict[str, Dict[str, Any]] = {}
        
    def load_config(self, config_path: Union[str, Path], config_dir: Optional[str] = None) -> Dict[str, Any]:
        """Load a Lua configuration file using cartographer_print_configuration.
        
        Args:
            config_path: Path or name of the Lua configuration file
            config_dir: Directory containing configuration files. If None, uses the directory of config_path
            
        Returns:
            Dictionary containing the parsed configuration
            
        Raises:
            FileNotFoundError: If config file not found
            subprocess.CalledProcessError: If cartographer_print_configuration fails
        """
        config_path = Path(config_path)
        
        if config_dir is None:
            config_dir = str(config_path.parent)
            config_basename = config_path.name
        else:
            config_basename = str(config_path)
            
        try:
            # Run cartographer_print_configuration command
            cmd = [
                "cartographer_print_configuration",
                f"-configuration_directories={config_dir}",
                f"-configuration_basename={config_basename}"
            ]
            result = subprocess.run(cmd, 
                                  capture_output=True, 
                                  text=True, 
                                  check=True)
            
            # Execute the expanded Lua configuration
            lua_code = result.stdout
            parsed_result = self.lua.execute(lua_code)
            
            # Convert Lua table to Python dict
            config_dict = self._lua_table_to_python(parsed_result)
            
            # Store the config
            self.configs["options"] = config_dict
            
            return config_dict
            
        except subprocess.CalledProcessError as e:
            raise RuntimeError(f"Failed to run cartographer_print_configuration: {e.stderr}")
        except Exception as e:
            raise RuntimeError(f"Error processing configuration: {str(e)}")
    
    def _lua_table_to_python(self, obj):
        if lupa.lua_type(obj) == 'table':
            # Check if the Lua table is array-like
            keys = list(obj.keys())
            if keys == list(range(1, len(keys) + 1)):
                # Convert to Python list if keys are consecutive integers starting from 1
                return [self._lua_table_to_python(obj[key]) for key in keys]
            else:
                # Convert to Python dictionary otherwise
                return {key: self._lua_table_to_python(obj[key]) for key in obj}
        return obj
    
    def _dict_to_lua_string(self, d: Dict[str, Any], indent: int = 0) -> str:
        """Convert a Python dictionary to a Lua table string representation.
        
        Args:
            d: Dictionary to convert
            indent: Current indentation level
            
        Returns:
            String containing Lua table representation
        """
        lines = []
        spaces = " " * (indent * 2)
        
        for k, v in d.items():
            if isinstance(v, dict):
                lines.append(f"{spaces}{k} = {{")
                lines.append(self._dict_to_lua_string(v, indent + 1))
                lines.append(f"{spaces}}},")
            elif isinstance(v, bool):
                lines.append(f"{spaces}{k} = {str(v).lower()},")
            elif isinstance(v, (int, float)):
                lines.append(f"{spaces}{k} = {v},")
            elif isinstance(v, str):
                lines.append(f'{spaces}{k} = "{v}",')
            else:
                lines.append(f"{spaces}{k} = {v},")
                
        return "\n".join(lines)
    
    def get_config(self, config_name: str) -> Optional[Dict[str, Any]]:
        """Get a loaded configuration by name.
        
        Args:
            config_name: Name of the configuration file
            
        Returns:
            Configuration dictionary if found, None otherwise
        """
        return self.configs.get(config_name)
    
    def to_lua_string(self, config_name: str) -> str:
        """Convert a loaded configuration back to Lua format.
        
        Args:
            config_name: Name of the configuration file
            
        Returns:
            String containing Lua configuration
        """
        config = self.get_config(config_name)
        if config is None:
            raise KeyError(f"Configuration not found: {config_name}")
            
        return f"options = {{\n{self._dict_to_lua_string(config, 1)}\n}}\n\nreturn options"
    
    def __str__(self):
        return self.to_
    
    def __getitem__(self, key: str) -> Any:
        """Get a value using dot notation for nested keys.
        
        Args:
            key: Key path using dot notation (e.g., "A.B.C")
            
        Returns:
            Value at the specified path
            
        Raises:
            KeyError: If the path doesn't exist
        """
        if "options" not in self.configs:
            raise KeyError("No configuration loaded")
            
        current = self.configs["options"]
        if not key:
            return current
            
        parts = key.split('.')
        
        # Navigate through nested dictionaries
        for i, part in enumerate(parts):
            if not isinstance(current, dict):
                raise KeyError(f"Cannot access '{part}' in '{'.'.join(parts[:i])}': not a dictionary")
            
            if part not in current:
                raise KeyError(f"Key not found: {key}")
                
            current = current[part]
            
        return current
    
    def __setitem__(self, key: str, value: Any):
        """Set a value using dot notation for nested keys.
        
        Args:
            key: Key path using dot notation (e.g., "A.B.C")
            value: Value to set
            
        Raises:
            KeyError: If the parent path doesn't exist
        """
        if "options" not in self.configs:
            self.configs["options"] = {}
            
        current = self.configs["options"]
        parts = key.split('.')
        
        # Navigate to the parent of the target key
        for part in parts[:-1]:
            if part not in current:
                current[part] = {}
            elif not isinstance(current[part], dict):
                raise KeyError(f"Cannot set key '{key}': parent is not a dictionary")
                
            current = current[part]
            
        # Set the value at the target key
        current[parts[-1]] = value
