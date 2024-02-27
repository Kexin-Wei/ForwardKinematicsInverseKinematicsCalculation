from lib.mesh.mesh import Mesh
from pathlib import Path
import matplotlib.pyplot as plt

data_path = Path("data").joinpath("stl_files")
probe_file = data_path.joinpath("EL96Probe.STL")


probe_stl = Mesh()
probe_stl.read_from_meshio(mesh_file=probe_file, file_format="stl")
probe_stl.plot()
plt.show()
