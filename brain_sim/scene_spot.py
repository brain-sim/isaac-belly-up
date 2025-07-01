import numpy as np
from isaacsim.core.api import World
from pxr import Sdf, UsdLux, UsdGeom, UsdShade
from isaacsim.core.prims import XFormPrim, GeometryPrim
from omni.isaac.core.objects import FixedCuboid
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
                            rendering_dt=1.0/100.0,
                            stage_units_in_meters=1.0)
        self.needs_reset = False

        self._subject = bsSpot()
        self._keyboard = bsKeyboard()

    def _add_light_to_stage(self, location):
        """
        A new stage does not have a light by default.  This function creates a spherical light
        """
        sphereLight = UsdLux.SphereLight.Define(get_current_stage(), Sdf.Path("/World/SphereLight"))
        sphereLight.CreateRadiusAttr(2)
        sphereLight.CreateIntensityAttr(100000)
        XFormPrim(str(sphereLight.GetPath())).set_world_poses(location)
        
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

        # Add semi-transparent plane at height 0.7
        
        # Create a thin cuboid to act as a semi-transparent plane
        transparent_plane = FixedCuboid(
            prim_path="/World/SemiTransparentPlane",
            name="transparent_plane",
            position=np.array([0, 0, 0.8]),
            scale=np.array([10.0, 10.0, 0.1]),  # Very thin in Z direction to act as a plane
            color=np.array([0.5, 0.5, 1.0])  # Light blue color
        )
        
        # Add the plane to the world scene
        self._world.scene.add(transparent_plane)
        
        # Set transparency via USD attributes
        plane_prim = transparent_plane.prim
        if plane_prim:
            # Create material for transparency
            material_path = "/World/Materials/TransparentMaterial"
            material = UsdShade.Material.Define(get_current_stage(), material_path)
            
            # Create shader
            shader = UsdShade.Shader.Define(get_current_stage(), material_path + "/Shader")
            shader.CreateIdAttr("UsdPreviewSurface")
            shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set((0.5, 0.5, 1.0))  # Light blue
            shader.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(0.01)  # Semi-transparent
            
            # Connect shader to material
            material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
            
            # Bind material to plane
            UsdShade.MaterialBindingAPI(plane_prim).Bind(material)

        self._subject.setup()
        self._keyboard.setup()
        self._world.reset()

        self._add_light_to_stage(np.array([[100, 0, 12]]))

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
        
        cur = self._keyboard.get_base_command()
        coef = 1
        self._subject.set_base_command([cur[0] * coef, cur[1] * coef, cur[2] * coef])
        self._subject.forward(dt)

    def cleanup(self):
        self._subject.cleanup()