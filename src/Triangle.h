#ifndef SCAN2CAD_RASTERIZER_TRIANGLE
#define SCAN2CAD_RASTERIZER_TRIANGLE

#include <Eigen/Dense>

template <typename Scalar>
class Triangle {

public:
    using Point2D = Eigen::Matrix<Scalar, 2, 1>;
    using Point3D = Eigen::Matrix<Scalar, 3, 1>;

    template <typename Index>
    struct BoundingBox {
        Index min_x = 0, max_x = 0;
        Index min_y = 0, max_y = 0;
    };

    Triangle(Point2D p0, Point2D p1, Point2D p2, Point3D z) : 
            m_p0(p0), m_p1(p1), m_p2(p2), m_z(z), 
            m_area(EdgeFunction(p0, p1, p2)) {}

    bool Contains(Point2D p) const {
        auto e01 = EdgeFunction(m_p0, m_p1, p);
        auto e12 = EdgeFunction(m_p1, m_p2, p);
        auto e20 = EdgeFunction(m_p2, m_p0, p);
        return (e01 >= 0) && (e12 >= 0) && (e20 >= 0);
    }

    template <typename Dimension>
    bool InImage(Dimension width, Dimension height) const {
        return 
            InImagePoint(m_p0, width, height) || 
            InImagePoint(m_p1, width, height) ||
            InImagePoint(m_p2, width, height);
    }

    Scalar Area() const { return m_area; }

    Point3D Bary(Point2D p) const {
        Point3D b;
        Scalar area = Area();
        b(0) = EdgeFunction(m_p1, m_p2, p) / area;
        b(1) = EdgeFunction(m_p2, m_p0, p) / area;
        b(2) = EdgeFunction(m_p0, m_p1, p) / area;
        return b;
    }

    Scalar Depth(Point2D p) const {
        return Depth(p, Bary(p));
    }

    // For pre-computations
    Scalar Depth(Point2D p, Point3D bary) const {
        Scalar div = bary(0) / m_z(0) + bary(1) / m_z(1) + bary(2) / m_z(2);
        return Scalar(1) / div;
    }

    template <typename Input>
    Input Interp(const std::array<Input, 3> &inputs, Point3D bary) const {
        return (
              bary.x() * inputs[0]
            + bary.y() * inputs[1]
            + bary.z() * inputs[2]
        );
    }

    template <typename Input>
    Input Interp(const std::array<Input, 3> &inputs, Point2D p) const {
        return Interp(inputs, Bary(p));
    }

    template <typename Dimension>
    BoundingBox<Dimension> Bounds(Dimension width, Dimension height) const {
        Scalar min_x = std::min({m_p0.x(), m_p1.x(), m_p2.x()});
        Scalar max_x = std::max({m_p0.x(), m_p1.x(), m_p2.x()});
        
        Scalar min_y = std::min({m_p0.y(), m_p1.y(), m_p2.y()});
        Scalar max_y = std::max({m_p0.y(), m_p1.y(), m_p2.y()});

        Scalar zero(0), x_up(width - 1), y_up(height - 1);
        min_x = std::max(zero, std::min(min_x, x_up));
        max_x = std::max(zero, std::min(max_x, x_up));
        min_y = std::max(zero, std::min(min_y, y_up));
        max_y = std::max(zero, std::min(max_y, y_up));

        BoundingBox<Dimension> b;
        b.min_x = std::floor(min_x);
        b.max_x = std::ceil(max_x);
        b.min_y = std::floor(min_y);
        b.max_y = std::ceil(max_y);
        return b;
    }

private:
    const Point2D m_p0, m_p1, m_p2;
    const Point3D m_z;
    const Scalar m_area;

    Scalar EdgeFunction(Point2D a, Point2D b, Point2D c) const {
         return (c(0) - a(0)) * (b(1) - a(1)) - (c(1) - a(1)) * (b(0) - a(0));
    }

    template <typename Dimension>
    bool InImagePoint(Point2D pt, Dimension width, Dimension height) const {
        return 
            (pt(0) < width) && (pt(0) >= 0) && 
            (pt(1) < height) && (pt(1) >= 0);
    }
};

#endif
