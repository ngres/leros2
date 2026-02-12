# Copyright 2024 The HuggingFace Inc. team. All rights reserved.
# Copyright 2026 Nicolas Gres
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Adapted from https://github.com/huggingface/lerobot/blob/a142c365dd6cb0c019d5031d612cd6da98e8562f/src/lerobot/scripts/lerobot_record.py

"""
Convert a ROS 2 bag file to a LeRobot dataset.

Example:

```shell
leros2-convert \
    --robot.type=ure \
    --dataset.repo_id=<my_username>/<my_dataset_name> \
    --bag_path=/path/to/your/rosbag \
    --task_topic=/task \
    --teleop.type=pose
```
"""
from functools import cached_property
from packaging.utils import _

from lerobot.teleoperators.utils import make_teleoperator_from_config
from lerobot.robots.utils import make_robot_from_config

from rosidl_runtime_py.utilities import get_message  # ty:ignore[unresolved-import]

from leros2.teleoperator import ROS2Teleoperator
from leros2.robot import ROS2Robot

import logging
from dataclasses import asdict, dataclass, field
from pathlib import Path
from pprint import pformat
from typing import Any

from lerobot.cameras import (
    CameraConfig,
)
from lerobot.cameras.opencv.configuration_opencv import OpenCVCameraConfig
from lerobot.cameras.realsense.configuration_realsense import RealSenseCameraConfig
from lerobot.configs import parser
from lerobot.datasets.image_writer import safe_stop_image_writer
from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.datasets.pipeline_features import (
    aggregate_pipeline_dataset_features,
    create_initial_features,
)
from lerobot.datasets.utils import build_dataset_frame, combine_feature_dicts
from lerobot.datasets.video_utils import VideoEncodingManager
from lerobot.processor import (
    RobotAction,
    RobotObservation,
    RobotProcessorPipeline,
    make_default_processors,
)
from lerobot.robots import ( 
    Robot,
    RobotConfig,
)
from lerobot.teleoperators import ( 
    Teleoperator,
    TeleoperatorConfig,
)
from lerobot.utils.constants import ACTION, OBS_STR
from lerobot.utils.control_utils import (
    init_keyboard_listener,
    is_headless,
    sanity_check_dataset_robot_compatibility,
)
from lerobot.utils.import_utils import register_third_party_devices
from lerobot.utils.utils import (
    init_logging,
    log_say,
)
from lerobot.utils.constants import HF_LEROBOT_HOME
import os
from shutil import rmtree

import rosbag2_py  # ty:ignore[unresolved-import]
from rclpy.serialization import deserialize_message  # ty:ignore[unresolved-import]


@dataclass
class DatasetRecordConfig:
    # Dataset identifier. By convention it should match '{hf_username}/{dataset_name}' (e.g. `lerobot/test`).
    repo_id: str
    # Root directory where the dataset will be stored (e.g. 'dataset/path').
    root: str | Path | None = None
    # Limit the frames per second.
    fps: int = 30
    # Encode frames in the dataset into video
    video: bool = True
    # Upload dataset to Hugging Face hub.
    push_to_hub: bool = True
    # Upload on private repository on the Hugging Face hub.
    private: bool = False
    # A short but accurate description of the task performed during the recording (e.g. "Pick the Lego block and drop it in the box on the right.")
    single_task: str | None = None
    # Add tags to your dataset on the hub.
    tags: list[str] | None = None
    # Number of subprocesses handling the saving of frames as PNG. Set to 0 to use threads only;
    # set to ≥1 to use subprocesses, each using threads to write images. The best number of processes
    # and threads depends on your system. We recommend 4 threads per camera with 0 processes.
    # If fps is unstable, adjust the thread count. If still unstable, try using 1 or more subprocesses.
    num_image_writer_processes: int = 0
    # Number of threads writing the frames as png images on disk, per camera.
    # Too many threads might cause unstable teleoperation fps due to main thread being blocked.
    # Not enough threads might cause low camera fps.
    num_image_writer_threads_per_camera: int = 4
    # Number of episodes to record before batch encoding videos
    # Set to 1 for immediate encoding (default behavior), or higher for batched encoding
    video_encoding_batch_size: int = 1
    # Rename map for the observation to override the image and state keys
    rename_map: dict[str, str] = field(default_factory=dict)


@dataclass
class RecordConfig:
    # Path to the bag file to convert
    bag_path: str
    # ROS 2 Robot
    robot: RobotConfig
    # Dataset config
    dataset: DatasetRecordConfig
    # ROS 2 Teleoperator
    teleop: TeleoperatorConfig
    # ROS 2 Event topic
    event_topic: str = "lerobot_event"
    # ROS 2 topic name with task descriptions (eache message will start a new episode in the dataset)
    task_topic: str | None = None
    # ROS 2 Clock topic (used to capture lerobot frames - if None, messages will be sampled at a fixed FPS rate)
    clock_topic: str | None = None
    # Maximum number of episodes to record
    max_episodes: int | None = None
    # Display all cameras on screen
    display_data: bool = False
    # Use vocal synthesis to read events.
    play_sounds: bool = False
    # Resume recording on an existing dataset.
    resume: bool = False


class DatasetConverter:

    robot: ROS2Robot
    teleop: ROS2Teleoperator
    teleop_action_processor: RobotProcessorPipeline[
        tuple[RobotAction, RobotObservation], RobotAction
    ]  # runs after teleop
    robot_observation_processor: RobotProcessorPipeline[
        RobotObservation, RobotObservation
    ]  # runs after robot
    dataset: LeRobotDataset
    event_topic: str | None = None
    task_topic: str | None = None
    clock_topic: str | None = None
    single_task: str | None = None
    max_episodes: int | None = None

    def __init__(self,
        robot: ROS2Robot,
        teleop: ROS2Teleoperator,
        teleop_action_processor: RobotProcessorPipeline[
            tuple[RobotAction, RobotObservation], RobotAction
        ],  # runs after teleop
        robot_observation_processor: RobotProcessorPipeline[
            RobotObservation, RobotObservation
        ],  # runs after robot
        dataset: LeRobotDataset,
        event_topic: str | None = None,
        task_topic: str | None = None,
        clock_topic: str | None = None,
        single_task: str | None = None,
        max_episodes: int | None = None,
    ):
        self.robot = robot
        self.teleop = teleop
        self.teleop_action_processor = teleop_action_processor
        self.robot_observation_processor = robot_observation_processor
        self.dataset = dataset
        self.event_topic = event_topic
        self.task_topic = task_topic
        self.clock_topic = clock_topic
        self.single_task = single_task
        self.max_episodes = max_episodes

    _raw_topic_data: dict[str, Any] = {}
    _parsed_topic_data: dict[str, Any] = {}
    _task: str | None = None
    _is_recording = False
    _has_frame = False
    _last_timestamp = 0
    _tasked_received = task_topic is None
    _num_episodes = 0
    _topic_types = []

    def _typename(self, topic_name):
        for topic_type in self._topic_types:
            if topic_type.name == topic_name:
                return topic_type.type
        raise ValueError(f"topic {topic_name} not in bag")

    def _deserialize_data(self, topic, data):
        msg_type = get_message(self._typename(topic))
        return deserialize_message(data, msg_type)

    def _save_episode(self):
        if not self._is_recording or not self._has_frame:
            return
        self.dataset.save_episode()
        self._num_episodes += 1
        self._last_timestamp = 0
        self._has_frame = False
        self._is_recording = False
        self._raw_topic_data.clear()
        self._parsed_topic_data.clear()

    @cached_property
    def topics(self) -> list[str]:
        return [
            t
            for t in [
                *self.robot.subscription_topics,
                *self.teleop.subscription_topics,
                self.event_topic,
                self.task_topic,
                self.clock_topic,
            ]
            if t is not None
        ]

    def convert(self, bag_path: str):
        reader = rosbag2_py.SequentialReader()
        reader.open(
            rosbag2_py.StorageOptions(uri=bag_path, storage_id="mcap"),
            rosbag2_py.ConverterOptions(
                input_serialization_format="cdr", output_serialization_format="cdr"
            ),
        )
        reader.set_filter(rosbag2_py.StorageFilter(topics=self.topics))

        self._topic_types = reader.get_all_topics_and_types()
        self._task = self.single_task

        try:
            while reader.has_next():
                if self.max_episodes and self._num_episodes >= self.max_episodes:
                    return

                topic, data, timestamp = reader.read_next()

                self._raw_topic_data[topic] = data
                self._parsed_topic_data.pop(topic, None) # Removed parsed message from cache

                # Check for task messages
                if self.task_topic is not None and topic == self.task_topic:
                    # Receive a new task to record
                    msg = self._deserialize_data(topic, data)
                    if self.single_task is None:
                        self._task = msg.data
                    print(f"episode {self._num_episodes} - new task: {self._task}")
                    # Save the previous episode (if recorded)
                    self._save_episode()
                    self._tasked_received = True
                    continue

                # Check for event messages
                if self.event_topic is not None and topic == self.event_topic:
                    msg = self._deserialize_data(topic, data)
                    match msg.data:
                        case "exit_early":
                            self._save_episode()
                            self._tasked_received = False
                            continue
                        case "rerecord_episode":
                            print("Rerecording episode...")
                            self._save_episode()
                            continue
                        case _:
                            continue

                if not self._is_recording:
                    if self._tasked_received and all(t in self._raw_topic_data for t in self.topics if t != self.event_topic and t != self.task_topic):
                        # Only start recording after all topics received a message
                        self._is_recording = True
                    else:
                        continue

                if self.clock_topic is None:
                    # Trigger a new frame after a certain time delta
                    if (timestamp - self._last_timestamp) < (10e8 // self.dataset.fps):
                        continue
                    self._last_timestamp = timestamp
                else:
                    # Trigger a new frame on every clock topic message
                    if topic != self.clock_topic:
                        continue

                # Parse all messages for the new frame
                for t, data in self._raw_topic_data.items():
                    if t not in self._parsed_topic_data:
                        self._parsed_topic_data[t] = self._deserialize_data(t, data)

                # Get robot observation
                obs = self.robot.get_state_from_topics(self._parsed_topic_data)

                # Applies a pipeline to the raw robot observation, default is IdentityProcessor
                obs_processed = self.robot_observation_processor(obs)

                observation_frame = build_dataset_frame(
                    self.dataset.features, obs_processed, prefix=OBS_STR
                )

                # Get action from teleop
                act = self.teleop.get_state_from_topics(self._parsed_topic_data)

                # Applies a pipeline to the action, default is IdentityProcessor
                action_values = self.teleop_action_processor((act, obs))

                # Write to dataset
                action_frame = build_dataset_frame(
                    self.dataset.features, action_values, prefix=ACTION
                )
                frame = {**observation_frame, **action_frame, "task": self._task}

                self.dataset.add_frame(frame)
                self._has_frame = True

            if self._has_frame:
                self._save_episode()

        except Exception as e:
            if self.dataset.image_writer is not None:
                print("Waiting for image writer to terminate...")
                self.dataset.image_writer.stop()
            raise e


@parser.wrap()
def record(cfg: RecordConfig) -> LeRobotDataset | None:
    init_logging()
    logging.info(pformat(asdict(cfg)))

    robot = make_robot_from_config(cfg.robot)
    if not isinstance(robot, ROS2Robot):
        raise ValueError("Robot must extend ROS2Robot")
    teleop = make_teleoperator_from_config(cfg.teleop)
    if not isinstance(teleop, ROS2Teleoperator):
        raise ValueError("Teleoperator must extend ROS2Teleoperator")

    teleop_action_processor, _robot_action_processor, robot_observation_processor = (
        make_default_processors()
    )

    dataset_features = combine_feature_dicts(
        aggregate_pipeline_dataset_features(
            pipeline=teleop_action_processor,
            initial_features=create_initial_features(
                action=teleop.action_features
            ),
            use_videos=cfg.dataset.video,
        ),
        aggregate_pipeline_dataset_features(
            pipeline=robot_observation_processor,
            initial_features=create_initial_features(
                observation=robot.observation_features
            ),
            use_videos=cfg.dataset.video,
        ),
    )

    dataset = None
    listener = None

    image_writer_threads = (
        cfg.dataset.num_image_writer_threads_per_camera * robot.image_feature_count
    )

    try:
        if cfg.resume:
            dataset = LeRobotDataset(
                cfg.dataset.repo_id,
                root=cfg.dataset.root,
                batch_encoding_size=cfg.dataset.video_encoding_batch_size,
            )

            if image_writer_threads > 0:
                dataset.start_image_writer(
                    num_processes=cfg.dataset.num_image_writer_processes,
                    num_threads=image_writer_threads,
                )
            sanity_check_dataset_robot_compatibility(
                dataset, robot, cfg.dataset.fps, dataset_features
            )
        else:
            root = Path(cfg.dataset.root) if cfg.dataset.root is not None else HF_LEROBOT_HOME / cfg.dataset.repo_id


            if os.path.exists(root) and not cfg.resume:
                if input("Dataset already exists locally. Do you want to overwrite it [y/N]? ").lower() == "y":
                    rmtree(root)
                else:
                    log_say("Conversion aborted", False, blocking=False)
                    return None

            # Create empty dataset or load existing saved episodes
            dataset = LeRobotDataset.create(
                cfg.dataset.repo_id,
                cfg.dataset.fps,
                root=cfg.dataset.root,
                robot_type=robot.name,
                features=dataset_features,
                use_videos=cfg.dataset.video,
                image_writer_processes=cfg.dataset.num_image_writer_processes,
                image_writer_threads=image_writer_threads,
                batch_encoding_size=cfg.dataset.video_encoding_batch_size,
            )

        robot.connect()
        if teleop is not None:
            teleop.connect()

        with VideoEncodingManager(dataset):
            converter = DatasetConverter(
                robot=robot,
                teleop=teleop,
                teleop_action_processor=teleop_action_processor,
                robot_observation_processor=robot_observation_processor,
                dataset=dataset,
                clock_topic=cfg.clock_topic,
                event_topic=cfg.event_topic,
                task_topic=cfg.task_topic,
                single_task=cfg.dataset.single_task,
                max_episodes=cfg.max_episodes
            )
            converter.convert(bag_path=cfg.bag_path)
    finally:
        log_say("Conversion finished", cfg.play_sounds, blocking=True)

        if dataset:
            dataset.finalize()

        if robot.is_connected:
            robot.disconnect()
        if teleop and teleop.is_connected:
            teleop.disconnect()

        if cfg.dataset.push_to_hub:
            dataset.push_to_hub(tags=cfg.dataset.tags, private=cfg.dataset.private)

        log_say("Exiting", cfg.play_sounds)
    return dataset


def main():
    register_third_party_devices()
    record()


if __name__ == "__main__":
    main()
