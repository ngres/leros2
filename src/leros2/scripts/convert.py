# Copyright 2024 The HuggingFace Inc. team. All rights reserved.
# Copyright 2025 Nicolas Gres
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
from lerobot.utils.visualization_utils import init_rerun, log_rerun_data


import rosbag2_py  # ty:ignore[unresolved-import]
from rclpy.serialization import deserialize_message  # ty:ignore[unresolved-import]


@dataclass
class DatasetRecordConfig:
    # Dataset identifier. By convention it should match '{hf_username}/{dataset_name}' (e.g. `lerobot/test`).
    repo_id: str
    # A short but accurate description of the task performed during the recording (e.g. "Pick the Lego block and drop it in the box on the right.")
    single_task: str
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
    # ROS 2 Task topic
    task_topic: str | None = None
    # ROS 2 Clock topic (used to capture lerobot frames - if None, messages will be sampled at a fixed FPS rate)
    clock_topic: str | None = None
    # Display all cameras on screen
    display_data: bool = False
    # Use vocal synthesis to read events.
    play_sounds: bool = False
    # Resume recording on an existing dataset.
    resume: bool = False


def typename(topic_name, topic_types):
    for topic_type in topic_types:
        if topic_type.name == topic_name:
            return topic_type.type
    raise ValueError(f"topic {topic_name} not in bag")


@safe_stop_image_writer
def record_loop(
    bag_path: str,
    robot: ROS2Robot,
    teleop: ROS2Teleoperator,
    teleop_action_processor: RobotProcessorPipeline[
        tuple[RobotAction, RobotObservation], RobotAction
    ],  # runs after teleop
    robot_observation_processor: RobotProcessorPipeline[
        RobotObservation, RobotObservation
    ],  # runs after robot
    dataset: LeRobotDataset,
    display_data: bool = False,
    event_topic: str | None = None,
    task_topic: str | None = None,
    clock_topic: str | None = None,
    single_task: str | None = None,
):
    """
    Record a bag file into a dataset.

    For every message in the provided `clock_topic`, the robot state and teleoperation actions are captured inside a new dataset frame.
    If no topic is provided, 
    If the provided `task_topic` is not None, the recording will begin when the first message is received on the task topic. Every further message will create a new episode.
    If the `task_topic` is None, the recording will begin immediately and only one episode will be captured.
    The `event_topic` is used to read events such as `"exit_early"` and `"rerecord_episode"`.

    Args:
        bag_path: Path to the bag file to record.
        robot: ROS2 robot to record.
        teleop: ROS2 teleoperator to record.
        teleop_action_processor: ROS2 teleoperator action processor.
        robot_observation_processor: ROS2 robot observation processor.
        dataset: Dataset to record into.
        display_data: Whether to display the data.
        clock_topic: Clock topic to use.
        event_topic: Event topic to use.
        task_topic: Task topic to use. If None, the recording will begin immediately.
        single_task: Default task to use if no task is received on the task topic.
    """
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag_path, storage_id="mcap"),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr", output_serialization_format="cdr"
        ),
    )

    topic_types = reader.get_all_topics_and_types()

    topics = [
        event_topic,
        *robot.subscription_topics,
        *teleop.subscription_topics,
    ]
    if task_topic is not None:
        topics.append(task_topic)
    if clock_topic is not None:
        topics.append(clock_topic)
    reader.set_filter(rosbag2_py.StorageFilter(topics=topics))

    print("meta", reader.get_metadata())

    raw_topic_data: dict[str, Any] = {}
    parsed_topic_data: dict[str, Any] = {}

    task = single_task

    is_recording = False
    is_saved = False

    last_timestamp = 0

    has_frame = False

    tasked_received = task_topic is None

    def deserialize_data(topic, data):
        msg_type = get_message(typename(topic, topic_types))
        return deserialize_message(data, msg_type)

    while reader.has_next():
        topic, data, timestamp = reader.read_next()

        # store raw message in raw_topic_data
        raw_topic_data[topic] = data
        parsed_topic_data.pop(topic, None)

        if topic == event_topic and event_topic is not None:
            msg = deserialize_data(topic, data)
            match msg.data:
                case "exit_early":
                    if is_recording and has_frame:
                        dataset.save_episode()
                        has_frame = False
                    is_recording = False
                    tasked_received = False
                    is_saved = True
                    last_timestamp = 0
                    continue
                case "rerecord_episode":
                    if dataset is not None:
                        dataset.clear_episode_buffer()
                    is_recording = True
                    last_timestamp = 0
                    continue
                case _:
                    continue

        if task_topic is not None and topic == task_topic:
            # receive a new task to record
            msg = deserialize_data(topic, data)
            if single_task is None:
                task = msg.data
            print("task")
            if is_recording and not is_saved and has_frame:
                # save the previous episode
                dataset.save_episode()
                is_saved = True
                has_frame = False
            tasked_received = True
            last_timestamp = 0
            continue

        # only record a new frame when the clock topic is received or the FPS threshold is reached
        if not is_recording:
            if tasked_received and all(t in raw_topic_data for t in topics if t != event_topic):
                # if no task_topic is provided, recording starts when all topics are saturated
                is_recording = True
            else:
                continue

        if clock_topic is not None and topic != clock_topic:
            continue

        if clock_topic is None:
            if (timestamp - last_timestamp) < (10e8 // dataset.fps):
                continue
            last_timestamp = timestamp

        for t, data in raw_topic_data.items():
            if t not in parsed_topic_data:
                parsed_topic_data[t] = deserialize_data(t, data)

        # Get robot observation
        obs = robot.get_state_from_topics(parsed_topic_data)

        # Applies a pipeline to the raw robot observation, default is IdentityProcessor
        obs_processed = robot_observation_processor(obs)

        observation_frame = build_dataset_frame(
            dataset.features, obs_processed, prefix=OBS_STR
        )

        # Get action from teleop
        act = teleop.get_state_from_topics(parsed_topic_data)

        # Applies a pipeline to the action, default is IdentityProcessor
        action_values = teleop_action_processor((act, obs))

        # Write to dataset
        action_frame = build_dataset_frame(
            dataset.features, action_values, prefix=ACTION
        )
        frame = {**observation_frame, **action_frame, "task": task}

        dataset.add_frame(frame)
        has_frame = True

        if display_data:
            log_rerun_data(observation=obs_processed, action=action_values)

        is_saved = False

    if is_recording and not is_saved and has_frame:
        dataset.save_episode()


@parser.wrap()
def record(cfg: RecordConfig) -> LeRobotDataset:
    init_logging()
    logging.info(pformat(asdict(cfg)))
    if cfg.display_data:
        init_rerun(session_name="converting")

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

    print("image_feature_count", robot.image_feature_count)

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
            record_loop(
                bag_path=cfg.bag_path,
                robot=robot,
                teleop=teleop,
                teleop_action_processor=teleop_action_processor,
                robot_observation_processor=robot_observation_processor,
                dataset=dataset,
                display_data=cfg.display_data,
                clock_topic=cfg.clock_topic,
                event_topic=cfg.event_topic,
                task_topic=cfg.task_topic,
                single_task=cfg.dataset.single_task,
            )
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
