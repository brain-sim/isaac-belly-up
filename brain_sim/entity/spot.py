import numpy as np
from isaacsim.robot.policy.examples.robots import SpotFlatTerrainPolicy
import omni.usd
from pxr import Usd, UsdGeom, Gf
from omni.isaac.core.utils.stage import get_current_stage
from typing import List, Tuple, Optional

class bsSpot:
    def __init__(self):
        self._spot = None
        self._spot_height = 0.3
        self._spot_start_position = np.array([0, 0, self._spot_height])
        self._base_command = [0.0, 0.0, 0.0]
        self._world = None

    def setup(self, start_position: List[float] = None):
        if start_position is not None:
            self._spot_start_position = np.array(start_position)
        
        self._spot = SpotFlatTerrainPolicy(
            prim_path="/World/Spot",
            name="Spot",
            position=self._spot_start_position,
            orientation=np.array([0, 1, 0, 0]),  # Quaternion (x, y, z, w)
        )

    def initialize(self):
        self._spot.initialize()
        # self._spot.post_reset()
        # self._spot.robot.set_joints_default_state(self._spot.default_pos)
        
    def forward(self, dt: float):
        self._spot.forward(dt, self._base_command)

    def set_base_command(self, command: List[float]):
        self._base_command = command

    def cleanup(self):
        if self._spot:
            try:
                if self._world and self._world.scene and hasattr(self._world.scene, 'remove_object'):
                    self._world.scene.remove_object("Spot", registry_only=False)
            except:
                pass
        self._spot = None

    def teleport(self, position: List[float]):
        if self._spot:
            new_position = position.copy()
            if len(new_position) == 2:
                new_position.append(self._spot_height)
            elif len(new_position) >= 3:
                new_position[2] = self._spot_height

            self.cleanup()
            self.setup(new_position)
            self.initialize()
            
        else:
            print("Spot robot is not initialized. Please call setup() first.")
            
    def get_position(self) -> Optional[Tuple[float, float, float]]:
        """
        Get the current position of the Spot robot from its body prim.
        
        Returns:
            Tuple[float, float, float]: (x, y, z) position of the robot, or None if not found
        """
        try:
            stage = get_current_stage()
            if not stage:
                print("No stage found")
                return None
                
            # Get the body prim path - adjust based on actual hierarchy
            body_prim_path = "/World/Spot/body"
            body_prim = stage.GetPrimAtPath(body_prim_path)
            
            # Get the transform
            xformable = UsdGeom.Xformable(body_prim)
            if not xformable:
                print(f"Prim at {body_prim_path} is not transformable")
                return None
                
            # Get the world transform matrix
            world_transform = xformable.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
            
            # Extract translation from the transformation matrix
            translation = world_transform.ExtractTranslation()
            
            return (float(translation[0]), float(translation[1]), float(translation[2]))
            
        except Exception as e:
            print(f"Error getting robot position: {e}")
            return None
            
    def get_position_2d(self) -> Optional[Tuple[float, float]]:
        """
        Get the current 2D position (x, y) of the Spot robot.
        
        Returns:
            Tuple[float, float]: (x, y) position of the robot, or None if not found
        """
        position_3d = self.get_position()
        if position_3d:
            return (position_3d[0], position_3d[1])
        return None