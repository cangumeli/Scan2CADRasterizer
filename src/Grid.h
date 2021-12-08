#ifndef SCAN2CAD_RASTERIZER_GRID_H
#define SCAN2CAD_RASTERIZER_GRID_H

#include <array>
#include <vector>

template<typename Scalar, size_t dims>
class Grid {

public:
    using Index = std::array<size_t, dims>;
    using Size = Index;

    explicit Grid(Size size, Scalar default_value=0) { 
        Init(size, default_value);
    }

    Grid() { Init({0}); }

    void Init(Size size, Scalar default_value=0) {
        // Set size
        m_size = size;

        // Set numel
        m_numel = 1;
        for (auto s : size) {
            m_numel *= s;
        }

        // Set strides
        int stride = 1;
        for (int i = dims - 1;  i > 0; --i) {
            m_strides[i] = stride;
            stride *= m_size[i];
        }
        m_strides[0] = stride;

        // Allocate and initialize data
        m_data.resize(m_numel, default_value);
    }

    Size GetSize() const { return m_size; }
    size_t GetNumEl() const { return m_numel; }
    Size GetStrides() const { return m_strides; }
    size_t GetDims() const { return dims; }
    bool IsEmpty() { return GetNumEl() == 0; }

    Size GetByteStrides() const {
        Size byte_strides = m_strides;
        for (auto &s : byte_strides) {
            s *= sizeof(Scalar);
        }
        return byte_strides;
    }

    Scalar *GetData() { return m_data.data(); }

    std::vector<Scalar> &GetVector() { return m_data; }

    Scalar &operator()(const Index index) {
        size_t index_1d = 0;
        for (int i = 0; i < dims; ++i) {
            index_1d += index[i] * m_strides[i];
        }
        //return m_data.at(index_1d);
        return m_data[index_1d];
    }

    void Fill(Scalar scalar) {
        for (auto &s : m_data) {
            s = scalar;
        }
    }

private:
    std::vector<Scalar> m_data;
    Size m_size;
    size_t m_numel;
    Size m_strides;
};

#endif
