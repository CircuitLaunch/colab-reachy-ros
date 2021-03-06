#!/usr/bin/env python3

import moveit_commander
import json
from pathlib import Path
from typing import List, Dict


POSE_FILE_DIR = Path(__file__).parents[2].absolute() / "config" / "moveit_joint_configurations"


def pose_file_paths(commander: moveit_commander.MoveGroupCommander):
    pose_file = POSE_FILE_DIR / f"{commander.get_name()}.json"
    temp_file = pose_file.with_suffix(pose_file.suffix + ".tmp")
    return (pose_file, temp_file)


def load_joint_configurations(
    commander: moveit_commander.MoveGroupCommander, joint_configurations: Dict[str, Dict[str, List[float]]]
):
    """
    Given a dict of joint configurations, as would be returned by MoveGroupCommander.get_remembered_joint_values(),
    load all the configurations in the dict into the provided MoveGroupCommander
    """

    for name, configuration in joint_configurations.items():
        commander.remember_joint_values(name, configuration)


def load_joint_configurations_from_file(commander: moveit_commander.MoveGroupCommander):
    """
    Given a path to a JSON file encoding a dict mapping dicts of joint configurations to move group names,
    load the commander with the configurations for its move group name.
    """

    pose_file, temp_file = pose_file_paths(commander)
    load_joint_configurations(commander, json.loads(pose_file.read_text()))


def save_joint_configurations_to_file(commander: moveit_commander.MoveGroupCommander):
    """
    Saves the MoveGroupCommander's remembered joint values to a JSON file at the given path.
    If the file already exists, it is overwritten.
    All joint configurations for this move group not loaded into the MoveGroupCommander will be overwritten and lost.
    """

    POSE_FILE_DIR.mkdir(parents=True, exist_ok=True)

    pose_file, temp_file = pose_file_paths(commander)

    temp_file.write_text(json.dumps(commander.get_remembered_joint_values(), indent=4, sort_keys=True))
    temp_file.rename(pose_file)
