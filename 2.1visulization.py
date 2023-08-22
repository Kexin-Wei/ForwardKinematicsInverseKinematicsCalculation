from pathlib import Path
from functools import partial
from lib.folder import URDFFolderMg
from yourdfpy import URDF
from yourdfpy.viz import viewer_callback, generate_joint_limit_trajectory

urdfFolderMg = URDFFolderMg(Path("data").joinpath("urdf_files"))
urdfFolderMg.getURDFFromAllDir()
urdfFolderMg.printURDFFiles()

urdf_path = urdfFolderMg.urdfs["pr2"]
assert isinstance(urdf_path, Path), "urdf_path is not a Path object"

robot_urdf = URDF.load(
    urdf_path, build_collision_scene_graph=True, load_collision_meshes=True
)

# Unable to resolve filename:meshes/base_v0/base_L.stl #FIXME
callback = None
loop_time = 6.0
trajectory = generate_joint_limit_trajectory(urdf_model=robot_urdf, loop_time=loop_time)
callback = partial(
    viewer_callback, urdf_model=robot_urdf, loop_time=loop_time, trajectory=trajectory
)

robot_urdf.show(
    collision_geometry=True,
    callback=callback,
)
