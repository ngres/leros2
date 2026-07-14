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

from mcap.reader import make_reader
from mcap_ros2.decoder import DecoderFactory
from tqdm import tqdm

from leros2.teleoperator import ROS2Teleoperator
from leros2.robot import ROS2Robot

import logging
from dataclasses import asdict, dataclass
from pathlib import Path
from pprint import pformat
from typing import Any

from lerobot.common.control_utils import sanity_check_dataset_robot_compatibility
from lerobot.configs import parser
from lerobot.configs.dataset import DatasetRecordConfig
from lerobot.datasets import (
    LeRobotDataset,
    VideoEncodingManager,
    aggregate_pipeline_dataset_features,
    create_initial_features,
    safe_stop_image_writer,
)
from lerobot.processor import (
    RobotAction,
    RobotObservation,
    RobotProcessorPipeline,
    make_default_processors,
)
from lerobot.robots import RobotConfig, make_robot_from_config
from lerobot.teleoperators import TeleoperatorConfig, make_teleoperator_from_config
from lerobot.utils.constants import ACTION, OBS_STR, HF_LEROBOT_HOME
from lerobot.utils.feature_utils import build_dataset_frame, combine_feature_dicts
from lerobot.utils.import_utils import register_third_party_plugins
from lerobot.utils.utils import (
    init_logging,
    log_say,
)
import os
from shutil import rmtree


@dataclass
class BaseConvertConfig:
    # Path to the bag file to convert
    bag_path: str
    # Dataset config
    dataset: DatasetRecordConfig
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

@dataclass(kw_only=True)
class ConvertConfig(BaseConvertConfig):
    # ROS 2 Robot
    robot: RobotConfig
    # ROS 2 Teleoperator
    teleop: TeleoperatorConfig


class _LazyMessage:
    """Defers CDR decoding of a bag message until its value is actually needed.

    Decoding ``sensor_msgs/Image`` is the most expensive step in the pipeline, so
    messages that get dropped between frame triggers should never be decoded. The
    decoded value is cached so a message spanning two frames is only decoded once.
    """

    __slots__ = ("_decoder", "_data", "_msg")

    def __init__(self, decoder, data):
        self._decoder = decoder
        self._data = data
        self._msg = None

    def get(self):
        if self._msg is None:
            self._msg = self._decoder(self._data)
        return self._msg


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

        self._topic_data: dict[str, _LazyMessage] = {}
        self._task: str | None = None
        self._is_recording = False
        self._has_frame = False
        self._last_timestamp = 0
        self._tasked_received = task_topic is None
        self._num_episodes = 0

    def _save_episode(self):
        if not self._is_recording or not self._has_frame:
            return
        self.dataset.save_episode()
        self._num_episodes += 1
        self._last_timestamp = 0
        self._has_frame = False
        self._is_recording = False

    def _delete_episode(self):
        if not self._is_recording:
            return
        self.dataset.clear_episode_buffer()
        self._last_timestamp = 0
        self._has_frame = False
        self._is_recording = False

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

    def _count_messages(self, bag_path: str) -> int | None:
        """Sum message counts for the subscribed topics from the mcap summary statistics."""
        try:
            with open(bag_path, "rb") as f:
                summary = make_reader(f).get_summary()
        except (OSError, ValueError):
            return None
        if summary is None or summary.statistics is None:
            return None
        topics = set(self.topics)
        return sum(
            count
            for channel_id, count in summary.statistics.channel_message_counts.items()
            if (channel := summary.channels.get(channel_id)) is not None
            and channel.topic in topics
        )

    def _iter_messages(self, bag_path: str):
        """Yield ``(topic, log_time_ns, _LazyMessage)`` without eagerly decoding.

        Uses the raw mcap reader so that expensive CDR decoding (images in
        particular) only happens for messages we decide to keep.
        """
        factory = DecoderFactory()
        decoders: dict[int, Any] = {}
        with open(bag_path, "rb") as f:
            for schema, channel, message in make_reader(f).iter_messages(
                topics=self.topics
            ):
                decoder = decoders.get(schema.id)
                if decoder is None:
                    decoder = factory.decoder_for(channel.message_encoding, schema)
                    decoders[schema.id] = decoder
                yield channel.topic, message.log_time, _LazyMessage(
                    decoder, message.data
                )

    @safe_stop_image_writer
    def convert(self, bag_path: str):
        self._task = self.single_task

        total = self._count_messages(bag_path)
        messages = tqdm(
            self._iter_messages(bag_path),
            total=total,
            unit="msg",
            desc="Converting bag",
        )
        for topic, timestamp, lazy_msg in messages:
            if self.max_episodes and self._num_episodes >= self.max_episodes:
                return

            self._topic_data[topic] = lazy_msg

            # Check for task messages
            if self.task_topic is not None and topic == self.task_topic:
                # Receive a new task to record
                if not self.single_task:
                    self._task = lazy_msg.get().data
                print(f"episode {self._num_episodes} - new task: {self._task}")
                # Save the previous episode (if recorded)
                self._save_episode()
                self._tasked_received = True
                continue

            # Check for event messages
            if self.event_topic is not None and topic == self.event_topic:
                match lazy_msg.get().data:
                    case "exit_early":
                        self._save_episode()
                        self._tasked_received = False
                        continue
                    case "delete_episode" | "rerecord_episode":
                        print("Deleting episode...")
                        self._delete_episode()
                        continue
                    case _:
                        continue

            if not self._is_recording:
                if self._tasked_received and all(t in self._topic_data for t in self.topics if t != self.event_topic and t != self.task_topic):
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

            # Decode only now that we've committed to emitting a frame. Skip the
            # control topics: the clock in particular only needs to have arrived,
            # never to be decoded.
            control_topics = (self.clock_topic, self.event_topic, self.task_topic)
            decoded = {
                t: lazy.get()
                for t, lazy in self._topic_data.items()
                if t not in control_topics
            }

            # Get robot observation
            obs = self.robot.get_state_from_topics(decoded)

            # Applies a pipeline to the raw robot observation, default is IdentityProcessor
            obs_processed = self.robot_observation_processor(obs)

            observation_frame = build_dataset_frame(
                self.dataset.features, obs_processed, prefix=OBS_STR
            )

            # Get action from teleop
            act = self.teleop.get_state_from_topics(decoded)

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

def convert(cfg: BaseConvertConfig, robot: ROS2Robot, teleop: ROS2Teleoperator) -> LeRobotDataset | None:
    init_logging()
    logging.info(pformat(asdict(cfg)))

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
            dataset = LeRobotDataset.resume(
                cfg.dataset.repo_id,
                root=cfg.dataset.root,
                image_writer_processes=cfg.dataset.num_image_writer_processes,
                image_writer_threads=image_writer_threads,
                batch_encoding_size=cfg.dataset.video_encoding_batch_size,
                rgb_encoder=cfg.dataset.rgb_encoder,
                depth_encoder=cfg.dataset.depth_encoder,
                streaming_encoding=cfg.dataset.streaming_encoding,
                encoder_queue_maxsize=cfg.dataset.encoder_queue_maxsize,
                encoder_threads=cfg.dataset.encoder_threads,
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
                rgb_encoder=cfg.dataset.rgb_encoder,
                depth_encoder=cfg.dataset.depth_encoder,
                streaming_encoding=cfg.dataset.streaming_encoding,
                encoder_queue_maxsize=cfg.dataset.encoder_queue_maxsize,
                encoder_threads=cfg.dataset.encoder_threads,
            )

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

        if cfg.dataset.push_to_hub:
            if dataset and dataset.num_episodes > 0:
                dataset.push_to_hub(tags=cfg.dataset.tags, private=cfg.dataset.private)
            else:
                logging.warning("No episodes saved — skipping push to hub")

        log_say("Exiting", cfg.play_sounds)
    return dataset


@parser.wrap()
def convert_cfg(cfg: ConvertConfig) -> LeRobotDataset | None:
    init_logging()
    logging.info(pformat(asdict(cfg)))

    robot = make_robot_from_config(cfg.robot)
    if not isinstance(robot, ROS2Robot):
        raise ValueError("Robot must extend ROS2Robot")
    teleop = make_teleoperator_from_config(cfg.teleop)
    if not isinstance(teleop, ROS2Teleoperator):
        raise ValueError("Teleoperator must extend ROS2Teleoperator")

    return convert(cfg, robot, teleop)


def main():
    register_third_party_plugins()
    convert_cfg()


if __name__ == "__main__":
    main()
