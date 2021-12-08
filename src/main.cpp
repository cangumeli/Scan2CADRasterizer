#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>
#include <pybind11/eigen.h>

#include "Rasterizer.h"

namespace py = pybind11;

template <typename DataType, size_t ndims = 1>
py::array_t<DataType> MakePyArray(Grid<DataType, ndims> &grid) {
    py::buffer_info info(
        grid.GetData(),                            /* Pointer to buffer */
        sizeof(DataType),                          /* Size of one scalar */
        py::format_descriptor<DataType>::format(), /* Python struct-style format descriptor */
        grid.GetDims(),                            /* Number of dimensions */
        grid.GetSize(),                            /* Buffer dimensions */
        grid.GetByteStrides()                      /* Strides (in bytes) for each index */
    );
    return py::array_t<DataType>(info);
}

void ParseAndAddModel(
    Rasterizer &self,
    const py::array_t<Rasterizer::Index> &faces,
    const py::array_t<Rasterizer::Scalar> &vertices,
    Rasterizer::MeshIndex idx,
    const py::array_t<Rasterizer::Scalar> &to_noc,
    const py::array_t<Rasterizer::Scalar> &face_normals
) {
    auto np_faces = faces.unchecked<2>();
    auto np_vertices = vertices.unchecked<2>();
    auto np_mat = to_noc.unchecked<2>();
    std::vector<Rasterizer::Vector3i> indices(np_faces.shape(0));
    std::vector<Rasterizer::Vector3> points(np_vertices.shape(0));
    Rasterizer::Matrix4 mat;
    // std::cout<<np_faces.shape(0)<<" "<<np_faces.shape(1)<<std::endl;
    
    for (ssize_t i = 0; i < np_faces.shape(0); ++i) {
        indices[i] = Rasterizer::Vector3i(
            np_faces(i, 0), np_faces(i, 1), np_faces(i, 2)
        );
    }
    for (ssize_t i = 0; i < np_vertices.shape(0); ++i) {
        // if (i == 0) printf("%d\n", sizeof(np_vertices(0, i)));
        points[i] = Rasterizer::Vector3(
            np_vertices(i, 0), np_vertices(i, 1), np_vertices(i, 2)
        );
    }
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            mat(i, j) = 
                static_cast<Rasterizer::Scalar>(np_mat(i, j));
        }
    }
    std::vector<Rasterizer::Normal> normals;
    if (self.HasNormals()) {
        auto np_normals = face_normals.unchecked<2>();
        normals.reserve(np_normals.shape(0));
        for (ssize_t i = 0; i < np_normals.shape(0); ++i) {
            normals.push_back(Rasterizer::Normal(
                np_normals(i, 0), np_normals(i, 1), np_normals(i, 2)
            ));
        }
    }
    self.AddModel(indices, points, idx, mat, normals);
}

PYBIND11_MODULE(scan2cad_rasterizer, m) {
    m.doc() = R"pbdoc(
        Documentation work in progress
        -----------------------

        .. currentmodule:: scan2cad_rasterizer

        .. autosummary::
           :toctree: _generate

           Rasterizer
    )pbdoc";

    py::class_<Rasterizer>(m, "Rasterizer", py::buffer_protocol())
            .def(py::init<Rasterizer::Scalar, Rasterizer::Scalar,
                          Rasterizer::Scalar, Rasterizer::Scalar>())

            .def(py::init<Rasterizer::Dimension, Rasterizer::Dimension,
                          Rasterizer::Scalar, Rasterizer::Scalar,
                          Rasterizer::Scalar, Rasterizer::Scalar>())
            
            .def(py::init<Rasterizer::Scalar, Rasterizer::Scalar,
                          Rasterizer::Scalar, Rasterizer::Scalar, bool>())

            .def(py::init<Rasterizer::Dimension, Rasterizer::Dimension,
                          Rasterizer::Scalar, Rasterizer::Scalar,
                          Rasterizer::Scalar, Rasterizer::Scalar, bool>())
            
            .def(py::init<Rasterizer::Scalar, Rasterizer::Scalar,
                          Rasterizer::Scalar, Rasterizer::Scalar,
                          bool, bool>())

            .def(py::init<Rasterizer::Dimension, Rasterizer::Dimension,
                          Rasterizer::Scalar, Rasterizer::Scalar,
                          Rasterizer::Scalar, Rasterizer::Scalar, 
                          bool, bool>())

            .def("rasterize", &Rasterizer::Rasterize)

            .def("clear_models", &Rasterizer::ClearModels)

            .def("set_colors", &Rasterizer::SetColors)

            .def(
                "render_colors",
                &Rasterizer::RenderColors,
                "Fill collor buffer",
                py::arg("normal_coef") = 0.3
            )

            /*
                Name accessor functions read since they copy
                the results into numpy arrays.
            */

            .def("read_depth", [](Rasterizer &self) {
                return MakePyArray<Rasterizer::Scalar, 2>(self.GetDepth());
            })

            .def("read_idx", [](Rasterizer &self) {
                return MakePyArray<Rasterizer::MeshIndex, 2>(self.GetID());
            })

            .def("read_noc", [](Rasterizer &self) {
                return MakePyArray<Rasterizer::Scalar, 3>(self.GetNOC());
            })

            .def("read_color", [](Rasterizer &self) {
                auto &grid = self.GetColorGrid();
                return MakePyArray<Rasterizer::Color::Scalar, 3>(grid);
            })

            .def("read_intrinsics", [](Rasterizer &self) {
                auto &mat = self.GetCameraMat();
                auto scalar_size = sizeof(Rasterizer::Scalar);

                // Initialize for col major, flip if row major
                std::array<size_t, 2> strides{scalar_size, 4 * scalar_size};
                if (Rasterizer::MatrixStorage == Eigen::RowMajor) {
                    std::swap(strides[0], strides[1]);
                }

                py::buffer_info info(
                    mat.data(),
                    sizeof(Rasterizer::Scalar),
                    py::format_descriptor<Rasterizer::Scalar>::format(),
                    2,
                    {4, 4},
                    strides
                );
                //return py::buffer()
                return py::array_t<Rasterizer::Scalar>(info);
            })

            .def("read_normals", [](Rasterizer &self) {
                auto &normals = self.GetNormalGrid();
                return MakePyArray<Rasterizer::Scalar, 3>(normals);
            })

            /*
            Return the necessary datatypes in numpy format for safety
            */
            .def_property_readonly("scalar_dtype", [] (Rasterizer &self) {
                std::string base = "float";
                return base + std::to_string(8 * sizeof(Rasterizer::Scalar));
            })

            .def_property_readonly("index_dtype", [](Rasterizer &self) {
                std::string base = "int";
                if (std::is_unsigned<Rasterizer::Index>::value) {
                    base = std::string("u") + base;
                }
                return base + std::to_string(8 * sizeof(Rasterizer::Index));
            })

            .def_property_readonly("idx_dtype", [](Rasterizer &self) {
                std::string base = "int";
                if (std::is_unsigned<Rasterizer::MeshIndex>::value) {
                    base = std::string("u") + base;
                }
                return base + 
                    std::to_string(8 * sizeof(Rasterizer::MeshIndex));
            })

            /*Rendering Functions*/

            .def_property_readonly("rendering_noc", [](Rasterizer &self) {
                return self.RenderingNOC();
            })

            .def_property_readonly("has_normals", [](Rasterizer &self) {
                return self.HasNormals();
            })

            .def_property_readonly("visible", &Rasterizer::GetVisible)

            .def("add_model", &ParseAndAddModel);

#ifdef VERSION_INFO
    m.attr("__version__") = VERSION_INFO;
#else
    m.attr("__version__") = "dev";
#endif
}
