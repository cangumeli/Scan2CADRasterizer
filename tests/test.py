import scan2cad_rasterizer as m
import numpy as np


r = m.Rasterizer(310., 310., 3., 5.)

f = np.array([[0, 0, 0]], dtype=r.index_dtype)
p = np.array([[3., 4., 5.]], dtype=r.scalar_dtype)
t = np.random.rand(4, 4)
r.add_model(f, p, 1, t)

idx = r.read_idx()
depth = r.read_depth()
noc = r.read_noc()
camera_mat = r.read_intrinsics()

print(
    idx.dtype, idx.shape, idx.strides, 
    np.zeros(idx.shape, dtype=idx.dtype).strides
)
print(
    depth.dtype, depth.shape, depth.strides, 
    np.zeros(depth.shape, dtype=depth.dtype).strides
)
print(
    noc.dtype, noc.shape, noc.strides, 
    np.zeros(noc.shape, dtype=noc.dtype).strides
)
