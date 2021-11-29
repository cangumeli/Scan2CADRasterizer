#include <cmath>
#include <iostream>
#include <numeric>
#include <tuple>
#include "Rasterizer.h"
#include "Triangle.h"
#define BOUND_CHECK 0
#define BOUND_CHECK_RAST 0


using Scalar = Rasterizer::Scalar;
using Dimension = Rasterizer::Dimension;

using Vector3 = Rasterizer::Vector3;
using Vector4 = Rasterizer::Vector4;
using Vector3i = Rasterizer::Vector3i;

using Matrix4 = Rasterizer::Matrix4;
using Matrix3 = Rasterizer::Matrix3;

using Face = Rasterizer::Face;

using Point2D = Triangle<Scalar>::Point2D;
using Point3D = Triangle<Scalar>::Point3D;

// Convert camera c{x, y} to an integer size
inline Dimension C2Dim(Scalar size) {
    return std::ceil(2 * size);
}

Rasterizer::Rasterizer(
    Scalar fx,
    Scalar fy,
    Scalar cx,
    Scalar cy,
    bool render_noc,
    bool has_normals
) : 
    m_height(C2Dim(cy)),
    m_width(C2Dim(cx)),
    m_depth({C2Dim(cy), C2Dim(cx)}, 0),
    m_id({C2Dim(cy), C2Dim(cx)}, 0),
    m_render_noc(render_noc),
    m_has_normals(has_normals) {

    InitCamera(fx, fy, cx, cy);
    if (render_noc) {
        m_noc.Init({m_height, m_width, 3});
    }
}

Rasterizer::Rasterizer(
    Dimension width,
    Dimension height,
    Scalar fx,
    Scalar fy,
    Scalar cx,
    Scalar cy,
    bool render_noc,
    bool has_normals
) : 
    m_height(height),
    m_width(width),
    m_depth({height, width}, 0),
    m_id({height, width}, 0),
    m_render_noc(render_noc),
    m_has_normals(has_normals) {

    InitCamera(fx, fy, cx, cy);
    if (render_noc) {
        m_noc.Init({height, width, 3});
    }
    if (has_normals) {
        m_normal_grid.Init({height, width, 3});
    }
}

void Rasterizer::InitCamera(Scalar fx, Scalar fy, Scalar cx, Scalar cy) {
    m_camera_mat(0, 0) = fx;
    m_camera_mat(1, 1) = fy;
    m_camera_mat(0, 2) = cx;
    m_camera_mat(1, 2) = cy;

    m_inv_camera_mat = m_camera_mat.inverse();
}

void Rasterizer::AddModel(
    const std::vector<Vector3i> &indices,
    const std::vector<Vector3> &points,
    MeshIndex idx,
    const Matrix4 &to_noc,
    const std::vector<Rasterizer::Normal> &normals
) {
    Mesh mesh(indices.size());
    for (int i = 0; i < mesh.size(); ++i) {
        const auto index = indices[i];
        mesh[i] = { 
            points[index[0]], 
            points[index[1]], 
            points[index[2]]
            /*points.at(index[0]),
            points.at(index[1]),
            points.at(index[2])*/
        };
    }
    m_meshes.push_back(std::move(mesh));

    m_mesh_ids.push_back(idx);
    m_to_nocs.push_back(to_noc);

    // Register vertex normals
    // std::cout<<m_has_normals<<"\n";
    if (m_has_normals) {
        m_normals.push_back(normals);
    }
        // simply set face normals

        // Compute vertex normals
        /*Normals vertex_normals(points.size(), Normal::Zero());
        std::vector<int> counts(vertex_normals.size(), 0);
        for (int i = 0; i < indices.size(); ++i) {
            const auto &normal = normals[i];
            const auto &index = indices[i];
            for (auto k : index) {
#if BOUND_CHECK
                counts.at(k)++;
                vertex_normals.at(k) += normal;
#else
                counts[k]++;
                vertex_normals[k] += normal;
#endif
            }
        }
        for (int i = 0; i < vertex_normals.size(); ++i) {
            if (counts[i] > 0) {
#if BOUND_CHECK
                vertex_normals.at(i) /= static_cast<Scalar>(counts.at(i));
#else
                vertex_normals[i] /= static_cast<Scalar>(counts[i]);
#endif
            }
        }

        // Register vertex normals to mesh
        Mesh mesh_normals(indices.size());
        for (int i = 0; i < indices.size(); ++i) {
            const auto &index = indices[i];
#if BOUND_CHECK
            mesh_normals[i] = {
                vertex_normals.at(index[0]),
                vertex_normals.at(index[1]),
                vertex_normals.at(index[2])
            };
#else
            mesh_normals[i] = {
                vertex_normals[index[0]],
                vertex_normals[index[1]],
                vertex_normals[index[2]]
            };
#endif
        }
        // printf("Size: %d\n", mesh_normals.size());
        m_normals.push_back(std::move(mesh_normals));
    } else {  // dummy
        m_normals.push_back(Mesh());
    }*/
}

void Rasterizer::ClearModels() {
    m_meshes.clear();
    m_to_nocs.clear();
    m_mesh_ids.clear();
    m_normals.clear();
}

inline std::tuple<Triangle<Scalar>, bool>
Perspective(const Face &face, const Matrix4 &intrinsics) {
    Point2D pts[3];
    Point3D zs;
    const auto min_z = std::numeric_limits<Scalar>::epsilon();
    Matrix3 intrinsics3 = intrinsics.block<3, 3>(0, 0);
    bool is_valid = false;

    for (int i = 0; i < 3; ++i) {
        const auto &p = face[i];
        Vector3 p_img = intrinsics3 * p;

        if(p_img.z() < min_z) {
            auto zero_pts = pts[0].Zero();
            auto zero_zs = zs.Zero();
            return std::make_tuple(
                Triangle<Scalar>(zero_pts, zero_pts, zero_pts, zero_zs),
                false
            );
        }
        /*bool front = (p_img.z() >= min_z)
        is_valid = is_valid || ;*/
        Point2D point(p_img.x() / p_img.z(), p_img.y() / p_img.z());
        pts[i] = point;
        zs(i) = p_img.z();
    }

    return std::make_tuple(
        Triangle<Scalar>(pts[0], pts[1], pts[2], zs), 
        true
    );
}

void Rasterizer::Rasterize() {
    // Reset the buffers
    if (m_render_noc) {
        m_noc.Fill(0);
    }
    m_depth.Fill(std::numeric_limits<Scalar>::max());
    m_id.Fill(0);
    if (m_has_normals) {
        m_normal_grid.Fill(0);
    }

    // Render mesh ids and z-buffer (depth)
    for (int i = 0; i < m_meshes.size(); ++i) {
        const auto &mesh = m_meshes[i];
        const auto id = m_mesh_ids[i];
        const auto &normals = m_normals[i];

        for (int k = 0; k < mesh.size(); ++k) {
            const auto &face = mesh[k];
            const auto perspective = Perspective(face, m_camera_mat);
            const auto triangle = std::get<0>(perspective);
            const auto is_valid = std::get<1>(perspective);

            if (!is_valid) {
                continue;
            }
            if (!triangle.InImage(m_width, m_height)) {
                continue;
            }

            const auto bbox = triangle.Bounds(m_width, m_height);
            for (auto y = bbox.min_y; y <= bbox.max_y; ++y) {
                for (auto x = bbox.min_x; x <= bbox.max_x; ++x) {
                    Point2D pixel(x, y);
                    if (triangle.Contains(pixel)) {
                        const auto bary = triangle.Bary(pixel);
                        const auto z = triangle.Depth(pixel, bary);
                        // Update images
                        if (z > 0 && z < m_depth({y, x})) {
                            m_depth({y, x}) = z;
                            m_id({y, x}) = id;
                            // Make the normal update
                            if (m_has_normals) {
                                const Normal normal = normals[k];
                                // printf("%d -> %d\n", k, normals.size());
                                /*const Normal normal = triangle.Interp(
#if BOUND_CHECK_RAST
                                    normals.at(k), bary
#else
                                    normals[k], bary
#endif
                                );*/
                                m_normal_grid({y, x, 0}) = normal.x();
                                m_normal_grid({y, x, 1}) = normal.y();
                                m_normal_grid({y, x, 2}) = normal.z();
                            }
                        }
                    }
                }
            }
        }
    }

    // Clean up background depths
    for (Index y = 0; y < m_height; ++y) {
        for (Index x = 0; x < m_width; ++x) {
            if (m_id({y, x}) == 0) {
                m_depth({y, x}) = 0;
            }
        }
    }

    // Don't do nocs
    if (!m_render_noc) {
        return;
    }

    // Compute nocs
    for (Index y = 0; y < m_height; ++y) {
        for (Index x = 0; x < m_width; ++x) {
            const auto id = m_id({y, x});
            if (id != 0) {
                // Find the to_noc transform
                // TODO: consider an O(1) representation?
                Matrix4 to_noc = Matrix4::Identity();
                for (int i = 0; i < m_mesh_ids.size(); ++i) {
                    if (m_mesh_ids[i] == id) {
                        to_noc = m_to_nocs[i];
                        break;
                    }
                }

                // Project depth to find the noc
                const auto z = m_depth({y, x});
                Vector4 point(x * z, y * z, z, 1);
                Vector4 noc = to_noc * m_inv_camera_mat * point;
                m_noc({y, x, 0}) = noc.x();
                m_noc({y, x, 1}) = noc.y();
                m_noc({y, x, 2}) = noc.z();
            }
        }
    }
}
