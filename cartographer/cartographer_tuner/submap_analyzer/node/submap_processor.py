"""Submap processor component for the submap analyzer node."""

import os
import asyncio
import logging
from typing import Dict, List, Optional, Any, Tuple, Set

# Configure logging first, before any other imports
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [%(levelname)s] [%(name)s] %(message)s',
    datefmt='%Y-%m-%d %H:%M:%S',
    force=True  # Force reconfiguration of the root logger
)
logger = logging.getLogger('submap_analyzer.node.processor')
logger.setLevel(logging.INFO)  # Ensure our processor logs are visible

# Now import other modules
from cartographer_ros_msgs.msg import SubmapEntry

from ..gui.message_types import MessageType, NewSubmapMessage

class SubmapProcessor:
    """Handles processing of submaps for the submap analyzer node."""
    
    def __init__(self, node):
        """Initialize the submap processor.
        
        Args:
            node: The parent SubmapAnalyzerNode
        """
        self.node = node
    
    async def process_single_submap(self, trajectory_id: int, submap_index: int, force_reload: bool = False):
        """Process a single submap.
        
        Args:
            trajectory_id: Trajectory ID of the submap
            submap_index: Index of the submap
            force_reload: Whether to force reloading even if already processed
        """
        submap_id = (trajectory_id, submap_index)
        success = False
        
        try:
            # Skip if already processed and not forcing reload
            if not force_reload and submap_id in self.node.processed_submaps:
                logger.debug(f"Skipping already processed submap {trajectory_id}_{submap_index}")
                return
            
            logger.info(f"[TRACK] Starting processing of submap {trajectory_id}_{submap_index}")
            
            # Query submap data from Cartographer
            submap_data = await self.node.query_submap(trajectory_id, submap_index)
            if not submap_data:
                logger.warning(f"[TRACK] Failed to query submap {trajectory_id}_{submap_index} from Cartographer")
                # Even if we failed to query, mark it as processed to avoid repeated failures
                self.node.processed_submaps.add(submap_id)
                return
                
            logger.info(f"[TRACK] Successfully queried submap {trajectory_id}_{submap_index} from Cartographer")
            logger.info(f"[TRACK] Saving submap {trajectory_id}_{submap_index} as PGM")
            
            # Save as PGM file
            saved_file = self.node.save_submap_as_pgm(submap_data)
            if not saved_file:
                self.node.error_count += 1
                self.node.last_error = f"Failed to save submap {trajectory_id}_{submap_index} as PGM"
                logger.error(f"[TRACK] {self.node.last_error}")
                # Even if we failed to save, mark it as processed to avoid repeated failures
                self.node.processed_submaps.add(submap_id)
                return
                
            logger.info(f"[TRACK] Successfully saved submap {trajectory_id}_{submap_index} to {saved_file}")
            
            # Mark as processed
            self.node.processed_submaps.add(submap_id)
            success = True
            
            # Notify GUI using the new dataclass
            message = NewSubmapMessage(
                trajectory_id=trajectory_id,
                submap_index=submap_index,
                file_path=saved_file,
                resolution=submap_data['resolution'],
                width=submap_data['width'],
                height=submap_data['height']
            )
            
            logger.info(f"[TRACK] Sending NEW_SUBMAP message to GUI for submap {trajectory_id}_{submap_index}")
            logger.info(f"[TRACK] Message details: file_path={saved_file}, resolution={submap_data['resolution']}, " +
                       f"width={submap_data['width']}, height={submap_data['height']}")
            
            self.node.update_queue.put((MessageType.NEW_SUBMAP, message))
            logger.info(f"[TRACK] NEW_SUBMAP message for {trajectory_id}_{submap_index} added to update queue")
            
            logger.info(f"[TRACK] Completed processing of submap {trajectory_id}_{submap_index}")
        
        except Exception as e:
            self.node.error_count += 1
            self.node.last_error = str(e)
            logger.error(f"[TRACK] Error processing submap {trajectory_id}_{submap_index}: {e}", exc_info=True)
            
            # Even if we failed, mark it as processed to avoid repeated failures
            if not success:
                self.node.processed_submaps.add(submap_id)
                logger.info(f"[TRACK] Marked submap {trajectory_id}_{submap_index} as processed despite failure")
        
        finally:
            # Remove from processing set regardless of success or failure
            if submap_id in self.node.processing_submaps:
                self.node.processing_submaps.remove(submap_id)
                logger.info(f"[TRACK] Removed submap {trajectory_id}_{submap_index} from processing_submaps set") 