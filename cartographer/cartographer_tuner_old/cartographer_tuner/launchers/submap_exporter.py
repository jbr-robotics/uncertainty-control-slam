#!/usr/bin/env python3

import os
import time
from pathlib import Path
from typing import Dict, List, Optional, Any, Tuple
import tempfile
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from launch import LaunchDescription
import numpy as np
from PIL import Image
import io
import asyncio
import threading
import gzip

from cartographer_ros_msgs.srv import SubmapQuery
from cartographer_ros_msgs.msg import SubmapList, SubmapEntry

from .base import BaseLauncher


class SubmapExporterNode(Node):
    """ROS2 node for exporting submaps."""
    
    def __init__(self, output_dir: str):
        """Initialize the submap exporter node.
        
        Args:
            output_dir: Directory to save exported submaps
        """
        super().__init__('submap_exporter')
        
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        
        # Create callback group for services
        self.callback_group = ReentrantCallbackGroup()
        
        # Create client for submap query service
        self.submap_query_client = self.create_client(
            SubmapQuery, 
            '/submap_query',
            callback_group=self.callback_group
        )
        
        # Create subscription for submap list
        self.submap_list = None
        self.submap_list_sub = self.create_subscription(
            SubmapList,
            '/submap_list',
            self.submap_list_callback,
            10,
            callback_group=self.callback_group
        )
        
        # Create rate object for timing
        self._loop_rate = self.create_rate(10, self.get_clock())  # 10Hz rate
        
        self.get_logger().info(f"Submap exporter initialized. Output directory: {self.output_dir}")
    
    def submap_list_callback(self, msg: SubmapList):
        """Callback for submap list messages."""
        self.submap_list = msg
        self.get_logger().info(f"Received submap list with {len(msg.submap)} submaps")
    
    def wait_with_rate(self, condition_func, timeout_sec: float = 10.0) -> bool:
        """Wait for a condition using the rate object.
        
        Args:
            condition_func: Function that returns True when condition is met
            timeout_sec: Timeout in seconds
            
        Returns:
            True if condition was met, False if timed out
        """
        start_time = time.time()
        while not condition_func():
            if time.time() - start_time > timeout_sec:
                return False
            self._loop_rate.sleep()
        return True
    
    async def wait_for_submap_list(self, timeout_sec: float = 10.0) -> bool:
        """Wait for submap list to be received.
        
        Args:
            timeout_sec: Timeout in seconds
            
        Returns:
            True if submap list was received, False if timed out
        """
        start_time = time.time()
        while self.submap_list is None:
            if time.time() - start_time > timeout_sec:
                self.get_logger().error(f"Timed out waiting for submap list after {timeout_sec} seconds")
                return False
            # Use asyncio.sleep for async compatibility
            await asyncio.sleep(0.1)
        return True
    
    async def wait_for_service(self, timeout_sec: float = 10.0) -> bool:
        """Wait for submap query service to be available.
        
        Args:
            timeout_sec: Timeout in seconds
            
        Returns:
            True if service is available, False if timed out
        """
        start_time = time.time()
        while not self.submap_query_client.wait_for_service(timeout_sec=1.0):
            if time.time() - start_time > timeout_sec:
                self.get_logger().error(f"Timed out waiting for submap_query service after {timeout_sec} seconds")
                return False
            self.get_logger().info("Waiting for submap_query service...")
            # Use asyncio.sleep for async compatibility
            await asyncio.sleep(0.1)
        return True
    
    async def query_submap(self, trajectory_id: int, submap_index: int) -> Optional[Dict]:
        """Query a submap from Cartographer.
        
        Args:
            trajectory_id: Trajectory ID
            submap_index: Submap index
            
        Returns:
            Submap data if successful, None otherwise
        """
        request = SubmapQuery.Request()
        request.trajectory_id = trajectory_id
        request.submap_index = submap_index
        
        self.get_logger().info(f"Querying submap: trajectory_id={trajectory_id}, submap_index={submap_index}")
        
        try:
            future = self.submap_query_client.call_async(request)
            # Wait for the future to complete using asyncio.sleep for async compatibility
            while not future.done():
                await asyncio.sleep(0.1)
            response = future.result()
            
            if response.status.code != 0:
                self.get_logger().error(f"Failed to query submap: {response.status.message}")
                return None
            
            # Check if we have any textures
            if not response.textures:
                self.get_logger().error(f"No textures in submap response")
                return None
                
            self.get_logger().info(f"Received submap: {len(response.textures)} textures")
            
            # Validate texture dimensions
            texture = response.textures[0]
            if texture.width <= 0 or texture.height <= 0:
                self.get_logger().error(f"Invalid texture dimensions: {texture.width}x{texture.height}")
                return None
            
            return {
                'trajectory_id': trajectory_id,
                'submap_index': submap_index,
                'version': response.submap_version,
                'textures': response.textures,
                'resolution': texture.resolution,
                'width': texture.width,
                'height': texture.height
            }
            
        except Exception as e:
            self.get_logger().error(f"Error querying submap: {e}")
            return None
    
    def save_submap_as_pgm(self, submap_data: Dict, texture_index: int = 0) -> str:
        """Save a submap texture as a PGM file.
        
        Args:
            submap_data: Submap data from query_submap
            texture_index: Index of texture to save (default: 0)
            
        Returns:
            Path to saved PGM file
        """
        texture = submap_data['textures'][texture_index]
        
        # Create output filename
        filename = f"submap_t{submap_data['trajectory_id']}_s{submap_data['submap_index']}.pgm"
        output_path = self.output_dir / filename
        
        # Create YAML metadata file
        yaml_path = output_path.with_suffix('.yaml')
        
        try:
            # Get dimensions from texture
            width = texture.width
            height = texture.height
            
            # Create a blank image (gray)
            image = Image.new('L', (width, height), 128)
            
            # If we have cells data, try to use it to fill in the image
            if hasattr(texture, 'cells') and texture.cells:
                self.get_logger().info(f"Found cells data for submap {submap_data['trajectory_id']}_{submap_data['submap_index']}")
                
                try:
                    # Try to decompress the data
                    decompressed_data = gzip.decompress(texture.cells)
                    data_size = len(decompressed_data)
                    self.get_logger().info(f"Decompressed data size: {data_size} bytes")
                    
                    # Check if the data size matches the expected size
                    expected_size = width * height
                    
                    if data_size == expected_size:
                        # Perfect match, use the data directly
                        # Create a numpy array from the decompressed data
                        cells_array = np.frombuffer(decompressed_data, dtype=np.uint8).reshape(height, width)
                        
                        # Create image from the array
                        image = Image.fromarray(cells_array)
                        self.get_logger().info(f"Created image from cells data with dimensions {height}x{width}")
                    elif data_size == expected_size * 2:
                        # This is the systematic case we've observed: 2 bytes per cell
                        # In Cartographer, each cell might be represented by 2 bytes:
                        # - First byte might be the occupancy value
                        # - Second byte might be a confidence/weight value or other metadata
                        
                        self.get_logger().info(f"Data has 2 bytes per cell - extracting occupancy values")
                        
                        # Extract every other byte (the occupancy values)
                        # We take the first byte of each pair (0, 2, 4, ...)
                        cells_array = np.frombuffer(decompressed_data[::2], dtype=np.uint8)
                        
                        # Reshape to the correct dimensions
                        cells_array = cells_array.reshape(height, width)
                        
                        # Create image from the array
                        image = Image.fromarray(cells_array)
                        self.get_logger().info(f"Created image from occupancy values with dimensions {height}x{width}")
                    else:
                        # Unexpected data size - log detailed information for debugging
                        self.get_logger().warn(f"Unexpected data size: got {data_size} bytes, expected {expected_size} or {expected_size * 2}")
                        self.get_logger().info(f"Submap dimensions: {width}x{height}, data size: {data_size} bytes")
                        
                        # Try to interpret the data based on what we know
                        if data_size > 0:
                            # First, try to use the data directly with the provided dimensions
                            try:
                                # If data size is larger than expected, it might have additional metadata
                                # Try to extract just the portion we need
                                if data_size > expected_size:
                                    # Try to extract the first expected_size bytes
                                    cells_array = np.frombuffer(decompressed_data[:expected_size], dtype=np.uint8)
                                    cells_array = cells_array.reshape(height, width)
                                    image = Image.fromarray(cells_array)
                                    self.get_logger().info(f"Created image using first {expected_size} bytes of data")
                                else:
                                    # Data is smaller than expected - pad with gray (128)
                                    padded_data = np.ones(expected_size, dtype=np.uint8) * 128
                                    padded_data[:data_size] = np.frombuffer(decompressed_data, dtype=np.uint8)
                                    cells_array = padded_data.reshape(height, width)
                                    image = Image.fromarray(cells_array)
                                    self.get_logger().info(f"Created image by padding {data_size} bytes to {expected_size}")
                            except Exception as e:
                                self.get_logger().warn(f"Failed to create image using direct approach: {e}")
                                # Fall back to the fallback method
                                image = self._create_fallback_image(decompressed_data, width, height, expected_size)
                        else:
                            self.get_logger().warn("Empty decompressed data, using blank image")
                except Exception as e:
                    self.get_logger().error(f"Error processing cells data: {e}")
                    # Continue with the blank image
            
            # Save the image as PGM
            image.save(output_path)
            
            # Write YAML metadata
            with open(yaml_path, 'w') as f:
                f.write(f"image: {filename}\n")
                f.write(f"resolution: {texture.resolution}\n")
                f.write(f"origin: [{texture.slice_pose.position.x}, {texture.slice_pose.position.y}, {texture.slice_pose.position.z}]\n")
                f.write(f"negate: 0\n")
                f.write(f"occupied_thresh: 0.65\n")
                f.write(f"free_thresh: 0.196\n")
            
            self.get_logger().info(f"Saved submap to {output_path}")
            return str(output_path)
            
        except Exception as e:
            self.get_logger().error(f"Error saving submap as PGM: {e}")
            return ""
    
    def _create_fallback_image(self, decompressed_data, width, height, expected_size):
        """Create a fallback image when other methods fail.
        
        Args:
            decompressed_data: The decompressed data bytes
            width: The expected width
            height: The expected height
            expected_size: The expected data size
            
        Returns:
            A PIL Image object
        """
        self.get_logger().info("Using fallback image creation method")
        
        try:
            # Try to use the raw data directly
            raw_array = np.frombuffer(decompressed_data, dtype=np.uint8)
            
            # Pad or truncate to match expected size
            if len(raw_array) < expected_size:
                # Pad with gray (128)
                padded_array = np.ones(expected_size, dtype=np.uint8) * 128
                padded_array[:len(raw_array)] = raw_array
                raw_array = padded_array
            elif len(raw_array) > expected_size:
                # Truncate
                raw_array = raw_array[:expected_size]
            
            # Reshape to expected dimensions
            cells_array = raw_array.reshape(height, width)
            image = Image.fromarray(cells_array)
            self.get_logger().info(f"Created fallback image with dimensions {height}x{width}")
            return image
        except Exception as e:
            self.get_logger().warn(f"Fallback image creation failed: {e}")
            # Last resort: return a blank image
            return Image.new('L', (width, height), 128)
    
    async def export_all_submaps(self) -> List[str]:
        """Export all available submaps as PGM files.
        
        Returns:
            List of paths to saved PGM files
        """
        # Wait for submap list and service
        if not await self.wait_for_service():
            self.get_logger().error("Failed to connect to submap query service")
            return []
        
        if not await self.wait_for_submap_list():
            self.get_logger().error("Failed to receive submap list")
            return []
        
        saved_files = []
        total_submaps = len(self.submap_list.submap)
        self.get_logger().info(f"Starting export of {total_submaps} submaps")
        
        # Process each submap
        for i, submap_entry in enumerate(self.submap_list.submap):
            try:
                trajectory_id = submap_entry.trajectory_id
                submap_index = submap_entry.submap_index
                
                # Log progress
                self.get_logger().info(f"Processing submap {i+1}/{total_submaps}: trajectory={trajectory_id}, index={submap_index}")
                
                # Query submap data
                submap_data = await self.query_submap(trajectory_id, submap_index)
                
                if submap_data:
                    # Save each texture as PGM
                    for texture_index in range(len(submap_data['textures'])):
                        try:
                            saved_file = self.save_submap_as_pgm(submap_data, texture_index)
                            if saved_file:  # Only add if file was successfully saved
                                saved_files.append(saved_file)
                        except Exception as e:
                            self.get_logger().error(f"Error saving texture {texture_index} for submap {trajectory_id}_{submap_index}: {e}")
                else:
                    self.get_logger().warn(f"Failed to query submap {trajectory_id}_{submap_index}")
                
                # Add a small delay between submaps to avoid overwhelming the system
                await asyncio.sleep(0.1)
                
            except Exception as e:
                self.get_logger().error(f"Error processing submap {i}: {e}")
        
        self.get_logger().info(f"Export complete. Exported {len(saved_files)} submap textures out of {total_submaps} submaps")
        return saved_files


class SubmapExporterLauncher(BaseLauncher):
    """Launcher for exporting Cartographer submaps as PGM files.
    
    This launcher connects to a running Cartographer node, queries all available
    submaps, and exports them as PGM files with YAML metadata.
    """
    
    @classmethod
    def _register_params(cls):
        """Register parameters for this launcher."""
        cls.register_parameter(
            name="output_dir",
            param_type=str,
            required=False,
            help="Directory to save exported submaps",
            default=None
        )
        cls.register_parameter(
            name="timeout",
            param_type=float,
            required=False,
            help="Timeout in seconds for waiting for services and data",
            default=30.0
        )
    
    def __init__(self, **kwargs):
        """Initialize the submap exporter launcher."""
        super().__init__(**kwargs)
        
        # Create output directory if not specified
        if self._output_dir is None:
            self.output_dir = tempfile.mkdtemp(prefix="cartographer_submaps_")
            print(f"No output directory specified. Using temporary directory: {self.output_dir}")
        else:
            self.output_dir = self._output_dir
            Path(self.output_dir).mkdir(parents=True, exist_ok=True)
        
        self.timeout = self._timeout
    
    def generate_launch_description(self) -> LaunchDescription:
        """Generate launch description for the submap exporter.
        
        This launcher doesn't use the launch system, so this method returns an empty
        launch description.
        """
        return LaunchDescription([])
    
    def run(self) -> None:
        """Run the submap exporter.
        
        This method initializes ROS, creates the submap exporter node, exports all
        submaps, and then shuts down ROS.
        """
        # Initialize ROS
        rclpy.init()
        
        try:
            # Create node
            node = SubmapExporterNode(self.output_dir)
            
            # Spin in a separate thread
            spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
            spin_thread.start()
            
            # Create asyncio event loop
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            
            try:
                # Run the export
                saved_files = loop.run_until_complete(node.export_all_submaps())
                print(f"Exported {len(saved_files)} submap textures")
                print(f"Submap export complete. Submaps saved to: {self.output_dir}")
            finally:
                loop.close()
                
        finally:
            # Ensure ROS is shutdown
            rclpy.shutdown()
            if 'spin_thread' in locals():
                spin_thread.join()


# Create main function for command-line usage
main = SubmapExporterLauncher.generate_main()

if __name__ == "__main__":
    main()
