import numpy as np
from isaacsim.core.api import World
from pxr import Sdf, UsdLux
from isaacsim.core.prims import XFormPrim
from isaacsim.core.utils.stage import get_current_stage

from typing import Tuple, List, Optional
from entity.spot import bsSpot
from utility.keyboard_manager import bsKeyboard


class SceneSpot:
    """
    Encapsulates scene entities for the spot in maze environment.
    """
    
    def __init__(self):

        self._world = World(physics_dt=1.0/200.0,
                            rendering_dt=1.0/30.0,
                            stage_units_in_meters=1.0)
        self.needs_reset = False

        self._subject = bsSpot()
        self._keyboard = bsKeyboard()

    def _add_light_to_stage(self):
        """
        A new stage does not have a light by default.  This function creates a spherical light
        """
        sphereLight = UsdLux.SphereLight.Define(get_current_stage(), Sdf.Path("/World/SphereLight"))
        sphereLight.CreateRadiusAttr(2)
        sphereLight.CreateIntensityAttr(100000)
        XFormPrim(str(sphereLight.GetPath())).set_world_poses(np.array([[6.5, 0, 12]]))
        
    def create_scene(self):

        self.cleanup()

        self._world.scene.add_default_ground_plane(
            z_position=0,
            name="default_ground_plane",
            prim_path="/World/defaultGroundPlane",
            static_friction=0.2,
            dynamic_friction=0.2,
            restitution=0.01,
        )

        self._subject.setup()
        self._keyboard.setup()
        self._world.reset()

        self._add_light_to_stage()

    def initialize(self):
        
        if self._subject:
            self._subject.initialize()
        else:
            print("Spot robot is not created. Please call create_scene() first.")
        self._world.add_physics_callback("scene_spot", callback_fn=self.advance)
    
    def advance(self, dt: float):

        if self.needs_reset:
            self._world.reset(True)
            self.initialize()
            self.needs_reset = False
            return
        
        self._subject.set_base_command(self._keyboard.get_base_command())
        self._subject.forward(dt)

    def cleanup(self):
        self._subject.cleanup()