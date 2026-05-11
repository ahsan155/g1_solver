import os
import time

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory

from .joint_table_target import target_waypoints
from .solver_ros import ArmJointCmdBridge, left_arm_vector_from_joint_states
from tesseract_robotics.tesseract_command_language import (
    AnyPoly_as_CompositeInstruction,
    AnyPoly_wrap_CompositeInstruction,
    AnyPoly_wrap_ProfileDictionary,
    CartesianWaypoint,
    CartesianWaypointPoly_wrap_CartesianWaypoint,
    CompositeInstruction,
    InstructionPoly_as_MoveInstructionPoly,
    JointWaypoint,
    JointWaypointPoly_wrap_JointWaypoint,
    MoveInstruction,
    MoveInstructionPoly_wrap_MoveInstruction,
    MoveInstructionType_FREESPACE,
    ProfileDictionary,
    WaypointPoly_as_StateWaypointPoly,
)
from tesseract_robotics.tesseract_common import (
    FilesystemPath,
    GeneralResourceLocator,
    Isometry3d,
    ManipulatorInfo,
    Quaterniond,
    Translation3d,
)
from tesseract_robotics.tesseract_environment import AnyPoly_wrap_EnvironmentConst, Environment
from tesseract_robotics.tesseract_task_composer import TaskComposerDataStorage, TaskComposerPluginFactory
from tesseract_robotics_viewer import TesseractViewer


class G1SolverApp:
    def __init__(self):
        self.task_composer_filename = os.environ["TESSERACT_TASK_COMPOSER_CONFIG_FILE"]
        self.config = self._load_solver_config()
        self.robot_cfg = self.config["robot"]
        self.manip_cfg = self.config["manipulator"]
        self.planner_cfg = self.config["planner"]

        rclpy.init()
        self.ros_bridge = ArmJointCmdBridge()

        self.locator = GeneralResourceLocator()
        self.t_env = self._init_environment()
        self.manip_info = self._build_manipulator_info()

        self.viewer = TesseractViewer()
        self.viewer.update_environment(self.t_env, [0, 0, 0])
        self.viewer.start_serve_background()

        self.joint_names = list(self.t_env.getActiveJointNames())
        self.joint_positions = np.zeros(len(self.joint_names), dtype=np.float64)
        self.viewer.update_joint_positions(self.joint_names, self.joint_positions)
        self.t_env.setState(self.joint_names, self.joint_positions)

        self.kg = self.t_env.getKinematicGroup(self.manip_info.manipulator)
        self.left_arm_names = list(self.t_env.getGroupJointNames(self.manip_info.manipulator))

        self.hold_seconds = float(self.planner_cfg["hold_seconds"])
        self._require_joint_feedback_or_exit()
        self.current_left_arm = self._sync_from_joint_feedback()
        self.orientation = self._compute_end_effector_orientation(self.current_left_arm)

        self.factory = TaskComposerPluginFactory(FilesystemPath(self.task_composer_filename), self.locator)
        self.task = self.factory.createTaskComposerNode(self.planner_cfg["pipeline"])
        self.output_key = self.task.getOutputKeys().get("program")
        self.input_key = self._resolve_input_key()

        self.profiles = ProfileDictionary()
        self.environment_anypoly = AnyPoly_wrap_EnvironmentConst(self.t_env)
        self.profiles_anypoly = AnyPoly_wrap_ProfileDictionary(self.profiles)
        self.task_executor = self.factory.createTaskComposerExecutor("TaskflowExecutor")

    def _load_solver_config(self):
        pkg_share = get_package_share_directory("g1_solver")
        config_file = os.path.join(pkg_share, "config", "solver_config.yaml")
        with open(config_file, "r", encoding="utf-8") as f:
            return yaml.safe_load(f)

    def _init_environment(self):
        urdf_url = self.robot_cfg["urdf_package_url"]
        srdf_url = self.robot_cfg["srdf_package_url"]
        urdf_fname = FilesystemPath(self.locator.locateResource(urdf_url).getFilePath())
        srdf_fname = FilesystemPath(self.locator.locateResource(srdf_url).getFilePath())
        env = Environment()
        assert env.init(urdf_fname, srdf_fname, self.locator)
        return env

    def _build_manipulator_info(self):
        manip_info = ManipulatorInfo()
        manip_info.tcp_frame = self.manip_cfg["end_effector"]
        manip_info.manipulator = self.manip_cfg["arm_group"]
        manip_info.working_frame = self.manip_cfg["parent_frame"]
        return manip_info

    def _require_joint_feedback_or_exit(self):
        timeout_s = float(self.planner_cfg["joint_states_timeout_s"])
        if not self.ros_bridge.wait_for_feedback(timeout_s=timeout_s):
            self.shutdown()
            raise RuntimeError("No /joint_states feedback received within timeout")

    def _sync_from_joint_feedback(self):
        rclpy.spin_once(self.ros_bridge, timeout_sec=0.2)
        left_arm_pos = left_arm_vector_from_joint_states(self.left_arm_names, self.ros_bridge.latest_joint_map)
        for i, name in enumerate(self.joint_names):
            if name in self.ros_bridge.latest_joint_map:
                self.joint_positions[i] = self.ros_bridge.latest_joint_map[name]
        self.t_env.setState(self.joint_names, self.joint_positions)
        self.viewer.update_joint_positions(self.joint_names, self.joint_positions)
        return np.array(left_arm_pos, dtype=np.float64)

    def _compute_end_effector_orientation(self, left_arm_pos):
        fk = self.kg.calcFwdKin(left_arm_pos)
        transform = fk[self.manip_info.tcp_frame]
        print(f"{self.manip_info.tcp_frame} matrix (from current /joint_states left arm)", transform.matrix())
        return transform.rotation()

    def _resolve_input_key(self):
        keys = self.task.getInputKeys()
        if keys.has("planning_input"):
            return keys.get("planning_input")
        if keys.has("program"):
            return keys.get("program")
        raise RuntimeError("Unknown pipeline input keys")

    def _build_segment_program(self, target_xyz):
        x, y, z = target_xyz
        start_wp = JointWaypoint(self.left_arm_names, self.current_left_arm)
        start_instruction = MoveInstruction(
            JointWaypointPoly_wrap_JointWaypoint(start_wp),
            MoveInstructionType_FREESPACE,
            "DEFAULT",
        )
        target_wp = CartesianWaypoint(
            Isometry3d.Identity() * Translation3d(x, y, z) * Quaterniond(self.orientation)
        )
        target_instruction = MoveInstruction(
            CartesianWaypointPoly_wrap_CartesianWaypoint(target_wp),
            MoveInstructionType_FREESPACE,
            "DEFAULT",
        )

        program = CompositeInstruction("DEFAULT")
        program.setManipulatorInfo(self.manip_info)
        program.appendMoveInstruction(MoveInstructionPoly_wrap_MoveInstruction(start_instruction))
        program.appendMoveInstruction(MoveInstructionPoly_wrap_MoveInstruction(target_instruction))
        return program

    def _plan_segment(self, target_xyz):
        program = self._build_segment_program(target_xyz)
        task_data = TaskComposerDataStorage()
        task_data.setData(self.input_key, AnyPoly_wrap_CompositeInstruction(program))
        task_data.setData("environment", self.environment_anypoly)
        task_data.setData("profiles", self.profiles_anypoly)

        future = self.task_executor.run(self.task.get(), task_data)
        future.wait()
        return future

    def _extract_planned_states(self, results):
        planned_states = []
        planned_times = []
        for instr in results:
            if not instr.isMoveInstruction():
                continue
            move_instr = InstructionPoly_as_MoveInstructionPoly(instr)
            waypoint = move_instr.getWaypoint()
            if waypoint.isStateWaypoint():
                state_wp = WaypointPoly_as_StateWaypointPoly(waypoint)
                planned_states.append(np.array(state_wp.getPosition().flatten(), dtype=np.float64))
                planned_times.append(float(state_wp.getTime()))
        return planned_states, planned_times

    def _publish_planned_segment(self, planned_states, planned_times):
        for state_idx, q in enumerate(planned_states):
            if state_idx > 0:
                dt = max(0.0, planned_times[state_idx] - planned_times[state_idx - 1])
                time.sleep(dt)
            self.ros_bridge.publish_left_arm_state(self.left_arm_names, q)
            self.viewer.update_joint_positions(self.left_arm_names, q)

        self.current_left_arm = planned_states[-1]
        self.t_env.setState(self.left_arm_names, self.current_left_arm)
        self.viewer.update_joint_positions(self.left_arm_names, self.current_left_arm)

    def run(self):
        for i, target_xyz in enumerate(target_waypoints):
            x, y, z = target_xyz
            future = self._plan_segment(target_xyz)
            if not future.context.isSuccessful():
                raise RuntimeError(f"Planning task failed at waypoint {i}: ({x:.3f}, {y:.3f}, {z:.3f})")

            results = AnyPoly_as_CompositeInstruction(future.context.data_storage.getData(self.output_key))
            planned_states, planned_times = self._extract_planned_states(results)
            if len(planned_states) == 0:
                raise RuntimeError(f"No final state waypoint found at segment {i}")

            self._publish_planned_segment(planned_states, planned_times)
            print(f"Reached waypoint {i}: ({x:.3f}, {y:.3f}, {z:.3f}), holding {self.hold_seconds:.1f}s")
            time.sleep(self.hold_seconds)

        input("press enter to exit")

    def shutdown(self):
        try:
            self.ros_bridge.destroy_node()
        finally:
            rclpy.shutdown()


def main():
    app = G1SolverApp()
    try:
        app.run()
    finally:
        app.shutdown()


if __name__ == "__main__":
    main()
