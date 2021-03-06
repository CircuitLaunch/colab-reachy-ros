#!/usr/bin/env python3

import moveit_commander
import json
from pathlib import Path
from typing import List, Dict


POSE_FILE = Path(__file__).parents[2].absolute() / "config" / "moveit_joint_configurations.json"
POSE_TEMPFILE = POSE_FILE.with_suffix(POSE_FILE.suffix + ".tmp")


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

    joint_configurations = json.loads(POSE_FILE.read_text())
    load_joint_configurations(commander, joint_configurations[commander.get_name()])


def save_joint_configurations_to_file(commander: moveit_commander.MoveGroupCommander):
    """
    Saves the MoveGroupCommander's remembered joint values to a JSON file at the given path.
    If the file already exists, it is overwritten.
    All joint configurations for this move group not loaded into the MoveGroupCommander will be overwritten and lost.
    """
    try:
        joint_configurations = json.loads(POSE_FILE.read_text())
    except FileNotFoundError:
        joint_configurations = {}

    joint_configurations[commander.get_name()] = commander.get_remembered_joint_values()
    POSE_TEMPFILE.write_text(json.dumps(joint_configurations, indent=4, sort_keys=True))
    POSE_TEMPFILE.rename(POSE_FILE)
