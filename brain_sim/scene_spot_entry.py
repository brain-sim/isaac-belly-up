from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

from scene_spot import SceneSpot

class ExampleEntry():
    def __init__(self):
        self._scene = None

    def setup_scenario(self):
        print("Setup scenario")
        self._scene = SceneSpot()
        self._scene.create_scene()
        self._scene.initialize()

if __name__ == "__main__":

    # Create and setup the scenario
    entry = ExampleEntry()
    entry.setup_scenario()
    
    # Ensure the scene was created successfully
    if entry._scene is None:
        print("Error: Scene was not created properly")
        simulation_app.close()
        exit(1)
    
    # Get the world instance from the scene
    while simulation_app.is_running():
        entry._scene._world.step(render=True)
        if entry._scene._world.is_stopped():
                entry._scene.needs_reset = True
