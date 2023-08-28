import open3d
from visual_utils import open3d_vis_utils as V

NPY_PATH = "/path/to/my_data.npy"
data = np.load(NPY_PATH)
V.draw_scenes(points=data)
