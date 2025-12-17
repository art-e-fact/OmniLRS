__author__ = "Antoine Richard"
__copyright__ = "Copyright 2023-24, Space Robotics Lab, SnT, University of Luxembourg, SpaceR"
__license__ = "BSD 3-Clause"
__version__ = "2.0.0"
__maintainer__ = "Antoine Richard"
__email__ = "antoine.richard@uni.lu"
__status__ = "development"

from src.environments_wrappers.sdg.lunaryard_sdg import SDG_Lunaryard
from src.environments_wrappers.sdg.lunaryard_sdg_slam import SDG_SLAM_Lunaryard
from src.environments_wrappers.sdg.lunalab_sdg import SDG_Lunalab
from src.configurations.auto_labeling_confs import AutoLabelingConf, CameraConf
from src.labeling.auto_label import AutonomousLabeling

from isaacsim.core.api.world import World
from typing import Union
import omni


from src.environments_wrappers.ros2.robot_manager_ros2 import ROS_RobotManager
from src.environments_wrappers.ros2.simulation_manager_ros2 import ROS2_LabManagerFactory
from rclpy.executors import SingleThreadedExecutor as Executor
from threading import Thread
from src.environments_wrappers.ros2.lunaryard_ros2 import ROS_LunaryardManager
from src.environments.utils import set_moon_env_name
from src.physics.physics_scene import PhysicsSceneManager

from src.environments.lunaryard import LunaryardController



class SyntheticDataGeneration_LabManagerFactory:
    def __init__(self):
        self._lab_managers = {}

    def register(self, name, lab_manager):
        self._lab_managers[name] = lab_manager

    def __call__(self, cfg):
        return self._lab_managers[cfg["environment"]["name"]](
            **cfg["environment"],
            flares_settings=cfg["rendering"]["lens_flares"],
            camera_settings=cfg["mode"]["camera_settings"],
        )


SDG_LMF = SyntheticDataGeneration_LabManagerFactory()
SDG_LMF.register("Lunalab", SDG_Lunalab)
SDG_LMF.register("Lunaryard", SDG_Lunaryard)
SDG_LMF.register("Lunaryard_SLAM", SDG_SLAM_Lunaryard)

ROS2_LMF = ROS2_LabManagerFactory()
ROS2_LMF.register("Lunaryard_SLAM", ROS_LunaryardManager)


class SDG_SimulationManager:
    """ "
    Manages the simulation. This class is responsible for:
    - Initializing the simulation
    - Running the simulation
    - Cleaning the simulation"""

    def __init__(self, cfg, simulation_app) -> None:
        self.simulation_app = simulation_app
        self.generation_settings = cfg["mode"]["generation_settings"]
        self.timeline = omni.timeline.get_timeline_interface()
        self.world = World(stage_units_in_meters=1.0)
        self.physics_ctx = self.world.get_physics_context()
        self.physics_ctx.set_solver_type("PGS")
        # Lab manager thread
        self.world.reset()
        self.LC = SDG_LMF(cfg)
        self.LC.load()
        print("After lab is loaded")
        for i in range(100):
            self.world.step(render=True)
        print("After world reset")
        self.generation_settings.prim_path = self.LC.scene_name + "/" + self.generation_settings.prim_path
        self.AL = AutonomousLabeling(self.generation_settings)
        self.AL.load()
        self.count = 0
        # Randomize once to setup the camera mixer
        self.LC.randomize()


    def run_simulation(self) -> None:
        """
        Runs the simulation.
        """

        self.timeline.play()
        while self.simulation_app.is_running() and (self.count < self.generation_settings.num_images):
            self.world.step(render=True)
            if self.world.is_playing():
                try:
                    self.AL.record()
                    self.count += 1
                except:
                    pass
                self.LC.randomize()
        self.timeline.stop()


class SDG_SLAM_SimulationManager:

    def __init__(self, cfg, simulation_app) -> None:
        self.simulation_app = simulation_app
        self.generation_settings = cfg["mode"]["generation_settings"]
        self.timeline = omni.timeline.get_timeline_interface()
        self.world = World(stage_units_in_meters=1.0)
        self.physics_ctx = self.world.get_physics_context()
        self.physics_ctx.set_solver_type("PGS")
        
        self.count = 0


        # Lab manager thread
        self.LabManager = ROS2_LMF(
            cfg, is_simulation_alive=self.simulation_app.is_running, close_simulation=self.simulation_app.close
        )
        self.exec1 = Executor()
        self.exec1.add_node(self.LabManager)
        self.exec1_thread = Thread(target=self.exec1.spin, daemon=True, args=())
        self.exec1_thread.start()
        
        # Robot manager thread
        self.ROSRobotManager = ROS_RobotManager(cfg["environment"]["robots_settings"])
        self.exec2 = Executor()
        self.exec2.add_node(self.ROSRobotManager)
        self.exec2_thread = Thread(target=self.exec2.spin, daemon=True, args=())
        self.exec2_thread.start()
        # breakpoint()

        self.ROSRobotManager.RM.preload_robot(self.world)

        self.LabManager.LC = SDG_LMF(cfg)
        self.LabManager.LC.load()
        
        self.LabManager.LC.add_robot_manager(self.ROSRobotManager.RM)


        for i in range(100):
            self.world.step(render=True)
        self.world.reset()

    
    def run_simulation(self) -> None:
        self.timeline.play()
        while self.simulation_app.is_running() and (self.count < self.generation_settings.num_images):
            self.world.step(render=True)
            if self.world.is_playing():
                try:
                    if self.world.current_time_step_index == 0:
                        self.ROSRobotManager.reset()
                    self.ROSRobotManager.apply_modifications()

                    self.LabManager.LC.captureCamera()
                    self.count += 1
                except:
                    pass

        self.world.stop()
        self.timeline.stop()
