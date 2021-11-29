#ifndef SCAN2CAD_RASTERIZER_RASTERIZER_H
#define SCAN2CAD_RASTERIZER_RASTERIZER_H

#include <Eigen/Dense>
#include "Grid.h"


class Rasterizer {

public:
    using Scalar = float;
    using Index = size_t;
    using Dimension = Index;
    using MeshIndex = uint16_t;

    const static auto MatrixStorage = Eigen::ColMajor;
    using Matrix3 = Eigen::Matrix<Scalar, 3, 3, MatrixStorage>;
    using Matrix4 = Eigen::Matrix<Scalar, 4, 4, MatrixStorage>;
    
    using Vector3 = Eigen::Matrix<Scalar, 3, 1>;
    using Vector4 = Eigen::Matrix<Scalar, 4, 1>;
    using Vector3i = Eigen::Matrix<Index, 3, 1>;
    
    using Face = std::array<Vector3, 3>;
    using Mesh = std::vector<Face>;
    using Normal = Vector3;
    using Normals = std::vector<Normal>;

    using NOC = Grid<Scalar, 3>;
    using Depth = Grid<Scalar, 2>;
    using ID = Grid<MeshIndex, 2>;
    using NormalGrid = Grid<Scalar, 3>;

    Rasterizer(
        Scalar fx,
        Scalar fy,
        Scalar cx,
        Scalar cy,
        bool render_noc=true,
        bool has_normals=false
    );
    Rasterizer(
        Dimension width,
        Dimension height, 
        Scalar fx,
        Scalar fy,
        Scalar cx,
        Scalar cy,
        bool render_noc=true,
        bool has_normals=false
    );

    Matrix4 &GetCameraMat() { return m_camera_mat; }
    NOC &GetNOC() { return m_noc; }
    Depth &GetDepth() { return m_depth; }
    ID &GetID() { return m_id; }
    NormalGrid &GetNormalGrid() { return m_normal_grid; }

    void AddModel(
        const std::vector<Vector3i> &indices,
        const std::vector<Vector3> &points,
        MeshIndex idx,
        const Matrix4 &to_noc,
        const std::vector<Normal> &normals
    );
    void ClearModels();

    bool RenderingNOC() const { return m_render_noc; }
    bool HasNormals() const { return m_has_normals; }

    void Rasterize();

private:
    const Dimension m_height;
    const Dimension m_width;
    const bool m_render_noc;
    const bool m_has_normals;

    Matrix4 m_camera_mat = Matrix4::Identity();
    Matrix4 m_inv_camera_mat = Matrix4::Identity();
    void InitCamera(Scalar fx, Scalar fy, Scalar cx, Scalar cy);

    NOC m_noc;
    Depth m_depth;
    ID m_id;
    NormalGrid m_normal_grid;

    std::vector<Mesh> m_meshes;
    std::vector<MeshIndex> m_mesh_ids;
    std::vector<Matrix4> m_to_nocs;
    std::vector<Normals> m_normals;
};


#endif
