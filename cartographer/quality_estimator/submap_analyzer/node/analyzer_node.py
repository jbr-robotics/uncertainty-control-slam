"""Main ROS2 node for analyzing submaps in real-time."""

import os
import sys
import asyncio
import queue
import logging
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional, Any, Tuple, Set, Union

# Configure logging first, before any other imports
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [%(levelname)s] [%(name)s] %(message)s',
    datefmt='%Y-%m-%d %H:%M:%S',
    force=True  # Force reconfiguration of the root logger
)
logger = logging.getLogger('submap_analyzer.node')
logger.setLevel(logging.INFO)  # Ensure our node logs are visible

# Now import other modules
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from cartographer_ros_msgs.srv import SubmapQuery
from cartographer_ros_msgs.msg import SubmapList, SubmapEntry

from ...launchers.submap_exporter import SubmapExporterNode
from ..gui.message_types import (
    MessageType, 
    SubmapListUpdatedMessage, 
    NewSubmapMessage, 
    RequestSubmapMessage
)
from .submap_processor import SubmapProcessor

class SubmapAnalyzerNode(SubmapExporterNode):
    """ROS2 node for analyzing submaps in real-time.
    
    This class extends SubmapExporterNode to monitor and process submaps from Cartographer.
    It handles the communication with ROS2 and provides submaps to the GUI.
    """
    
    def __init__(self, output_dir: str, update_queue: queue.Queue, loop: asyncio.AbstractEventLoop):
        """Initialize the submap analyzer node.
        
        Args:
            output_dir: Directory to save exported submaps
            update_queue: Queue for communication with the GUI
            loop: Asyncio event loop to use for background tasks
        """
        print("[TRACK] SubmapAnalyzerNode.__init__ called")
        logger.info("[TRACK] Initializing SubmapAnalyzerNode")
        super().__init__(output_dir)
        logger.info("[TRACK] SubmapExporterNode initialized")
        
        # Communication with GUI
        self.update_queue = update_queue
        self.loop = loop
        logger.info("[TRACK] Communication variables set")
        
        # State tracking
        self.processed_submaps = set()  # Set of (trajectory_id, submap_index) tuples that have been processed
        self.processing_submaps = set()  # Set of (trajectory_id, submap_index) tuples currently being processed
        self.last_update_time = None
        self.error_count = 0
        self.last_error = None
        
        # Create components
        self.submap_processor = SubmapProcessor(self)
        
        # Set up periodic check for new submaps
        self.timer_callback_group = MutuallyExclusiveCallbackGroup()
        self.update_timer = self.create_timer(
            1.0, 
            self._check_for_new_submaps,
            callback_group=self.timer_callback_group
        )
        
        logger.info("Submap analyzer node initialized")
    
    def submap_list_callback(self, msg: SubmapList):
        """Process incoming submap list messages from Cartographer.
        
        This method is called whenever a new submap list is published by Cartographer.
        It updates the internal state and notifies the GUI.
        
        Args:
            msg: The submap list message from Cartographer
        """
        logger.info(f"[TRACK] Received submap list with {len(msg.submap)} submaps")
        
        # Track new submaps
        new_submaps_count = 0
        if self.submap_list:
            old_count = len(self.submap_list.submap)
            new_count = len(msg.submap)
            if new_count > old_count:
                new_submaps_count = new_count - old_count
                logger.info(f"[TRACK] Detected {new_submaps_count} new submaps (old: {old_count}, new: {new_count})")
        else:
            new_submaps_count = len(msg.submap)
            logger.info(f"[TRACK] First submap list received with {new_submaps_count} submaps")
        
        # Update internal state
        self.submap_list = msg
        self.last_update_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        logger.info(f"[TRACK] Updated internal submap_list and last_update_time")
        
        # Log at appropriate level
        if new_submaps_count > 0:
            logger.info(f"[TRACK] Received submap list with {len(msg.submap)} submaps ({new_submaps_count} new)")
        else:
            logger.debug(f"[TRACK] Received submap list update with {len(msg.submap)} submaps (no new)")
        
        # Extract all submap IDs from the message
        submap_ids = [(entry.trajectory_id, entry.submap_index) for entry in msg.submap]
        logger.info(f"[TRACK] Extracted {len(submap_ids)} submap IDs from message")
        logger.info(f"[TRACK] Submap IDs: {submap_ids}")
        
        # Notify GUI using the new dataclass with full submap list
        message = SubmapListUpdatedMessage(
            count=len(msg.submap),
            submap_ids=submap_ids
        )
        logger.info(f"[TRACK] Created SubmapListUpdatedMessage with {len(submap_ids)} submap IDs")
        
        self.update_queue.put((MessageType.SUBMAP_LIST_UPDATED, message))
        logger.info(f"[TRACK] Added SUBMAP_LIST_UPDATED message to update queue")
        
        # Schedule processing of any new submaps
        logger.info(f"[TRACK] Scheduling check for new submaps")
        self._check_for_new_submaps()
    
    def _check_for_new_submaps(self, skip_if_already_called: bool = False):
        """Periodically check for new submaps that need processing.
        
        This method is called by the timer and schedules processing of new submaps.
        
        Args:
            skip_if_already_called: If True, skip processing if this method has already been called in the current cycle
        """
        logger.info(f"[TRACK] Checking for new submaps (skip_if_already_called={skip_if_already_called})")
        
        # Skip if prerequisites aren't met
        if self.submap_list is None:
            logger.debug("[TRACK] No submap list available yet, skipping check")
            return
            
        # Find unprocessed submaps that aren't currently being processed
        new_submaps = [
            entry for entry in self.submap_list.submap
            if (entry.trajectory_id, entry.submap_index) not in self.processed_submaps
            and (entry.trajectory_id, entry.submap_index) not in self.processing_submaps
        ]
        
        logger.info(f"[TRACK] Found {len(new_submaps)} new submaps to process")
        logger.info(f"[TRACK] Current processed_submaps count: {len(self.processed_submaps)}")
        logger.info(f"[TRACK] Current processing_submaps count: {len(self.processing_submaps)}")
        
        if len(new_submaps) > 0:
            logger.info(f"[TRACK] New submap IDs: {[(s.trajectory_id, s.submap_index) for s in new_submaps]}")
        
        # Process new submaps if any
        if new_submaps:
            logger.info(f"[TRACK] Starting processing of {len(new_submaps)} new submaps")
            # Process each submap individually to allow concurrent processing
            for submap in new_submaps:
                submap_id = (submap.trajectory_id, submap.submap_index)
                logger.info(f"[TRACK] Adding submap {submap_id} to processing_submaps set")
                self.processing_submaps.add(submap_id)
                logger.info(f"[TRACK] Scheduling async processing of submap {submap_id}")
                asyncio.run_coroutine_threadsafe(
                    self.submap_processor.process_single_submap(
                        submap.trajectory_id, 
                        submap.submap_index
                    ), 
                    self.loop
                )
                logger.info(f"[TRACK] Successfully scheduled processing of submap {submap_id}")
        else:
            logger.info(f"[TRACK] No new submaps to process")
    
    async def run(self):
        """Run the submap analyzer node.
        
        This method starts the monitoring process and handles requests from the GUI.
        """
        print("[TRACK] SubmapAnalyzerNode.run called")
        logger.info("[TRACK] Starting submap analyzer")
        
        # Wait for prerequisites
        if not await self.wait_for_service() or not await self.wait_for_submap_list():
            self.error_count += 1
            self.last_error = "Failed to initialize submap analyzer"
            logger.error(f"[TRACK] {self.last_error}")
            sys.exit(1)
            
        logger.info("[TRACK] Submap analyzer ready - services and submap list available")
        
        # Process initial submaps
        if self.submap_list and self.submap_list.submap:
            initial_submaps = self.submap_list.submap
            logger.info(f"[TRACK] Processing {len(initial_submaps)} initial submaps")
            # Process each submap individually to allow concurrent processing
            for submap in initial_submaps:
                submap_id = (submap.trajectory_id, submap.submap_index)
                if submap_id not in self.processed_submaps and submap_id not in self.processing_submaps:
                    logger.info(f"[TRACK] Adding initial submap {submap_id} to processing_submaps set")
                    self.processing_submaps.add(submap_id)
                    logger.info(f"[TRACK] Starting processing of initial submap {submap_id}")
                    await self.submap_processor.process_single_submap(
                        submap.trajectory_id, 
                        submap.submap_index
                    )
                    logger.info(f"[TRACK] Completed processing of initial submap {submap_id}")
        
        # Main loop - process GUI requests
        logger.info("[TRACK] Entering main loop to process GUI requests")
        while rclpy.ok():
            try:
                # Process GUI requests
                queue_size = self.update_queue.qsize()
                if queue_size > 0:
                    logger.info(f"[TRACK] Found {queue_size} items in update queue")
                
                while not self.update_queue.empty():
                    request_type, data = self.update_queue.get_nowait()
                    logger.info(f"[TRACK] Processing request from GUI: type={request_type}")
                    
                    if request_type == MessageType.REQUEST_SUBMAP:
                        # Handle submap request
                        if isinstance(data, dict):
                            # Handle legacy dictionary format
                            trajectory_id = data['trajectory_id']
                            submap_index = data['submap_index']
                            logger.info(f"[TRACK] Received legacy format request for submap {trajectory_id}_{submap_index}")
                        else:
                            # Handle new dataclass format
                            trajectory_id = data.trajectory_id
                            submap_index = data.submap_index
                            logger.info(f"[TRACK] Received dataclass format request for submap {trajectory_id}_{submap_index}")
                            
                        submap_id = (trajectory_id, submap_index)
                        
                        # Check if this submap is already being processed
                        if submap_id in self.processing_submaps:
                            logger.info(f"[TRACK] Submap {trajectory_id}_{submap_index} is already being processed, skipping")
                            continue
                            
                        logger.info(f"[TRACK] Processing request for submap {trajectory_id}_{submap_index}")
                        logger.info(f"[TRACK] Adding submap {submap_id} to processing_submaps set")
                        self.processing_submaps.add(submap_id)
                        logger.info(f"[TRACK] Scheduling async processing of requested submap {submap_id}")
                        asyncio.run_coroutine_threadsafe(
                            self.submap_processor.process_single_submap(
                                trajectory_id,
                                submap_index,
                                True  # Always force reload for manual requests
                            ),
                            self.loop
                        )
                        logger.info(f"[TRACK] Successfully scheduled processing of requested submap {submap_id}")
            
            except Exception as e:
                self.error_count += 1
                self.last_error = str(e)
                logger.error(f"[TRACK] Error processing GUI requests: {e}", exc_info=True)
            
            # Sleep to avoid busy waiting
            await asyncio.sleep(0.1) 