# Scan2CADRasterizer

A simple Rasterizer for the [Scan2CAD](www.Scan2CAD.org) data. The project is based on the [Pybind11 CMake example](https://github.com/pybind/cmake_example).

To install this repository, simply run below for the installation:
```
$ git clone --recursive https://github.com/cangumeli/Scan2CADRasterizer.git
$ cd Scan2CADRasterizer
$ pip install .
```
You can now use the library by:
```
>>> from scan2cad_rasterizer import Rasterizer
```

For usage please check the [demo notebook](/tests/demo.ipynb).

This repo is created for the [ROCA](https://cangumeli.github.io/ROCA/) project. If you find this code helpful, please cite:
```
@article{gumeli2022roca,
  title={ROCA: Robust CAD Model Retrieval and Alignment from a Single Image},
  author={G{\"u}meli, Can and Dai, Angela and Nie{\ss}ner, Matthias},
  booktitle={Proc. Computer Vision and Pattern Recognition (CVPR), IEEE},
  year={2022}
}
```
