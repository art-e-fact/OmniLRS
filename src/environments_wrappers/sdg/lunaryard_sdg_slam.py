__author__ = "Antoine Richard; Bach Nguyen"
__copyright__ = "Copyright 2023-24, Space Robotics Lab, SnT, University of Luxembourg, SpaceR, Artefacts / Asteria ART"
__license__ = "BSD 3-Clause"
__version__ = "2.0.0"
__maintainer__ = "Antoine Richard; Bach Nguyen"
__email__ = "antoine.richard@uni.lu; bach@artefacts.com"
__status__ = "development"


from scipy.spatial.transform import Rotation as SSTR
import numpy as np

from isaacsim.core.api.world import World
from pxr import Gf
import omni

from src.environments.lunaryard import LunaryardController

from src.configurations.procedural_terrain_confs import TerrainManagerConf
from src.configurations.auto_labeling_confs import CameraConf
from src.configurations.rendering_confs import FlaresConf
from src.configurations.environments import LunaryardConf
from src.labeling.auto_label import AutonomousLabeling
from src.configurations.stellar_engine_confs import StellarEngineConf, SunConf
from assets import get_assets_path

from WorldBuilders.pxr_utils import setDefaultOps, addDefaultOps
from WorldBuilders.Types import Position_T, Orientation_T, UserRequest_T
from WorldBuilders.Types import UniformSampler_T, ImageClipper_T
from WorldBuilders.Types import Image_T, RollPitchYaw_T
from WorldBuilders.Mixer import RequestMixer

from pxr import UsdGeom, Gf


from isaacsim.sensors.camera import Camera
import matplotlib.pyplot as plt
import cv2
from typing import Dict


class SDG_SLAM_Lunaryard(LunaryardController):
    def __init__(
        self,
        lunaryard_settings: LunaryardConf = None,
        rocks_settings: dict = None,
        flares_settings: FlaresConf = None,
        terrain_manager: TerrainManagerConf = None,
        camera_settings: CameraConf = None,

        stellar_engine_settings: StellarEngineConf = None,
        sun_settings: SunConf = None,
        static_assets_settings: Dict = None,
        monitoring_cameras_settings: Dict = None,

        **kwargs,
    ) -> None:
        super().__init__(
            lunaryard_settings=lunaryard_settings,
            rocks_settings=rocks_settings,
            flares_settings=flares_settings,
            terrain_manager=terrain_manager,

            stellar_engine_settings=stellar_engine_settings,
            sun_settings=sun_settings,
            static_assets_settings=static_assets_settings,
            monitoring_cameras_settings=monitoring_cameras_settings,
            **kwargs
        )
        self.camera_settings = camera_settings
        self.terrain_settings = terrain_manager
        self.counter = 0
    
        self.camera = Camera(
            prim_path="/Robots/husky/d455/RSD455/Camera_OmniVision_OV9782_Color",
            frequency=30,
            resolution=(256,256)
        )
        self.camera.initialize()

    def load(self) -> None:
        super().load()
    
    def captureCamera(self) -> None:
        tmp = self.camera.get_current_frame()
        plt.imshow(tmp['rgb']); plt.show()

    
