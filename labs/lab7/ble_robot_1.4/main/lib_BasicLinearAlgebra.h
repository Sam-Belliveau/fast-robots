#pragma once

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "Arduino.h"
#include "Printable.h"

namespace BLA {

    template <typename DerivedType, int rows, int cols, typename DType>
    struct MatrixBase;

    template <int Rows, int Cols = 1, typename DType = float>
    class Matrix
        : public MatrixBase<Matrix<Rows, Cols, DType>, Rows, Cols, DType> {
      public:
        DType storage[Rows * Cols];

        DType &operator()(int i, int j = 0) {
            return storage[i * Cols + j];
        }
        DType operator()(int i, int j = 0) const {
            return storage[i * Cols + j];
        }

        Matrix() = default;

        template <typename... TAIL> Matrix(DType head, TAIL... args) {
            FillRowMajor(0, head, args...);
        }

        template <typename... TAIL>
        void FillRowMajor(int start_idx, DType head, TAIL... tail) {
            static_assert(
                Rows * Cols > sizeof...(TAIL),
                "Too many arguments passed to FillRowMajor"
            );

            (*this)(start_idx / Cols, start_idx % Cols) = head;

            FillRowMajor(++start_idx, tail...);
        }

        void FillRowMajor(int start_idx) {
            for (int i = start_idx; i < Rows * Cols; ++i) {
                (*this)(i / Cols, i % Cols) = 0.0;
            }
        }

        template <typename DerivedType>
        Matrix(const MatrixBase<DerivedType, Rows, Cols, DType> &mat) {
            static_cast<MatrixBase<Matrix<Rows, Cols, DType>, Rows, Cols, DType>
                            &>(*this) = mat;
        }

        Matrix &operator=(const Matrix &mat) {
            static_cast<MatrixBase<Matrix<Rows, Cols, DType>, Rows, Cols, DType>
                            &>(*this) = mat;
            return *this;
        }
    };

    template <int Rows, int Cols = 1, typename DType = float>
    class Zeros
        : public MatrixBase<Zeros<Rows, Cols, DType>, Rows, Cols, DType> {
      public:
        DType operator()(int i, int j = 0) const {
            return 0.0;
        }

        Zeros() = default;
    };

    template <int Rows, int Cols = 1, typename DType = float>
    class Ones : public MatrixBase<Ones<Rows, Cols, DType>, Rows, Cols, DType> {
      public:
        DType operator()(int i, int j = 0) const {
            return 1.0;
        }

        Ones() = default;
    };

    template <int Rows, int Cols = 1, typename DType = float>
    class Eye : public MatrixBase<Eye<Rows, Cols, DType>, Rows, Cols, DType> {
      public:
        DType operator()(int i, int j = 0) const {
            return i == j;
        }

        Eye() = default;
    };

    template <typename RefType, int Rows, int Cols>
    class RefMatrix : public MatrixBase<
                          RefMatrix<RefType, Rows, Cols>,
                          Rows,
                          Cols,
                          typename RefType::DType> {
        RefType &parent_;
        const int row_offset_;
        const int col_offset_;

      public:
        explicit RefMatrix(
            RefType &parent, int row_offset = 0, int col_offset = 0
        )
            : parent_(parent), row_offset_(row_offset),
              col_offset_(col_offset) {}

        typename RefType::DType &operator()(int i, int j) {
            return parent_(i + row_offset_, j + col_offset_);
        }
        typename RefType::DType operator()(int i, int j) const {
            return parent_(i + row_offset_, j + col_offset_);
        }

        template <typename MatType> RefMatrix &operator=(const MatType &mat) {
            static_cast<MatrixBase<
                RefMatrix<RefType, Rows, Cols>,
                Rows,
                Cols,
                typename RefType::DType> &>(*this) = mat;
            return *this;
        }
    };

    template <typename RefType>
    class MatrixTranspose : public MatrixBase<
                                MatrixTranspose<RefType>,
                                RefType::Cols,
                                RefType::Rows,
                                typename RefType::DType> {
        RefType &parent_;

      public:
        explicit MatrixTranspose(RefType &parent) : parent_(parent) {}

        typename RefType::DType &operator()(int i, int j) {
            return parent_(j, i);
        }
        typename RefType::DType operator()(int i, int j) const {
            return parent_(j, i);
        }

        template <typename MatType>
        MatrixTranspose &operator=(const MatType &mat) {
            return static_cast<MatrixBase<
                       MatrixTranspose<RefType>,
                       RefType::Cols,
                       RefType::Rows,
                       typename RefType::DType> &>(*this) = mat;
        }
    };

    template <typename LeftType, typename RightType>
    struct HorizontalConcat : public MatrixBase<
                                  HorizontalConcat<LeftType, RightType>,
                                  LeftType::Rows,
                                  LeftType::Cols + RightType::Cols,
                                  typename LeftType::DType> {
        const LeftType &left;
        const RightType &right;

        HorizontalConcat(const LeftType &l, const RightType &r)
            : left(l), right(r) {}

        typename LeftType::DType operator()(int row, int col) const {
            return col < LeftType::Cols ? left(row, col)
                                        : right(row, col - LeftType::Cols);
        }
    };

    template <typename TopType, typename BottomType>
    struct VerticalConcat : public MatrixBase<
                                VerticalConcat<TopType, BottomType>,
                                TopType::Rows + BottomType::Rows,
                                TopType::Cols,
                                typename TopType::DType> {
        const TopType &top;
        const BottomType &bottom;

        VerticalConcat(const TopType &t, const BottomType &b)
            : top(t), bottom(b) {}

        typename TopType::DType operator()(int row, int col) const {
            return row < TopType::Rows ? top(row, col)
                                       : bottom(row - TopType::Rows, col);
        }
    };

    template <int Rows, int Cols, typename DType, int TableSize>
    struct SparseMatrix : public MatrixBase<
                              SparseMatrix<Rows, Cols, DType, TableSize>,
                              Rows,
                              Cols,
                              DType> {
        DType end;

        static constexpr int Size = TableSize;

        struct Element {
            int row, col;
            DType val;

            Element() {
                row = col = -1;
            }

        } table[TableSize];

        DType &operator()(int row, int col) {
            int hash = (row * Cols + col) % TableSize;

            for (int i = 0; i < TableSize; i++) {
                Element &item = table[(hash + i) % TableSize];

                if (item.row == -1 || item.val == 0) {
                    item.row = row;
                    item.col = col;
                    item.val = 0;
                }

                if (item.row == row && item.col == col) {
                    return item.val;
                }
            }

            return end;
        }

        DType operator()(int row, int col) const {
            int hash = (row * Cols + col) % TableSize;

            for (int i = 0; i < TableSize; i++) {
                const Element &item = table[(hash + i) % TableSize];

                if (item.row == row && item.col == col) {
                    return item.val;
                }
            }

            return 0;
        }
    };

    template <int Dim, class DType>
    struct PermutationMatrix
        : public MatrixBase<PermutationMatrix<Dim, DType>, Dim, Dim, DType> {
        int idx[Dim];

        DType operator()(int row, int col) const {
            return idx[col] == row;
        }
    };

    template <class ParentType>
    struct LowerUnitriangularMatrix : public MatrixBase<
                                          LowerUnitriangularMatrix<ParentType>,
                                          ParentType::Rows,
                                          ParentType::Cols,
                                          typename ParentType::DType> {
        const ParentType &parent;

        LowerUnitriangularMatrix(const ParentType &obj) : parent(obj) {}

        typename ParentType::DType operator()(int row, int col) const {
            if (row > col) {
                return parent(row, col);
            } else if (row == col) {
                return 1;
            } else {
                return 0;
            }
        }
    };

    template <class ParentType>
    struct LowerTriangularMatrix : public MatrixBase<
                                       LowerTriangularMatrix<ParentType>,
                                       ParentType::Rows,
                                       ParentType::Cols,
                                       typename ParentType::DType> {
        const ParentType &parent;

        LowerTriangularMatrix(const ParentType &obj) : parent(obj) {}

        typename ParentType::DType operator()(int row, int col) const {
            if (row >= col) {
                return parent(row, col);
            } else {
                return 0;
            }
        }
    };

    template <class ParentType>
    struct UpperTriangularMatrix : public MatrixBase<
                                       UpperTriangularMatrix<ParentType>,
                                       ParentType::Rows,
                                       ParentType::Cols,
                                       typename ParentType::DType> {
        const ParentType &parent;

        UpperTriangularMatrix(const ParentType &obj) : parent(obj) {}

        typename ParentType::DType operator()(int row, int col) const {
            if (row <= col) {
                return parent(row, col);
            } else {
                return 0;
            }
        }
    };

} // namespace BLA

namespace BLA {

    template <typename DerivedType, int rows, int cols, typename d_type>
    struct MatrixBase : public Printable {
      public:
        constexpr static int Rows = rows;
        constexpr static int Cols = cols;
        using DType = d_type;

        DType &operator()(int i, int j = 0) {
            return static_cast<DerivedType *>(this)->operator()(i, j);
        }

        DType operator()(int i, int j = 0) const {
            return static_cast<const DerivedType *>(this)->operator()(i, j);
        }

        MatrixBase() = default;

        template <typename MatType>
        MatrixBase(const MatrixBase<MatType, Rows, Cols, DType> &mat) {
            for (int i = 0; i < rows; ++i) {
                for (int j = 0; j < cols; ++j) {
                    static_cast<DerivedType &>(*this)(i, j) = mat(i, j);
                }
            }
        }

        MatrixBase &operator=(const MatrixBase &mat) {
            for (int i = 0; i < rows; ++i) {
                for (int j = 0; j < cols; ++j) {
                    static_cast<DerivedType &>(*this)(i, j) = mat(i, j);
                }
            }

            return static_cast<DerivedType &>(*this);
        }

        template <typename MatType>
        MatrixBase &
        operator=(const MatrixBase<MatType, Rows, Cols, DType> &mat) {
            for (int i = 0; i < rows; ++i) {
                for (int j = 0; j < cols; ++j) {
                    static_cast<DerivedType &>(*this)(i, j) = mat(i, j);
                }
            }

            return static_cast<DerivedType &>(*this);
        }

        DerivedType &operator=(DType elem) {
            for (int i = 0; i < rows; ++i) {
                for (int j = 0; j < cols; ++j) {
                    static_cast<DerivedType &>(*this)(i, j) = elem;
                }
            }

            return static_cast<DerivedType &>(*this);
        }

        void Fill(const DType &val) {
            *this = val;
        }

        template <typename DestType> Matrix<Rows, Cols, DestType> Cast() {
            Matrix<Rows, Cols, DestType> ret;

            for (int i = 0; i < rows; ++i) {
                for (int j = 0; j < cols; ++j) {
                    ret(i, j) = (DestType)(*this)(i, j);
                }
            }

            return ret;
        }

        template <int SubRows, int SubCols>
        RefMatrix<DerivedType, SubRows, SubCols>
        Submatrix(int row_start, int col_start) {
            return RefMatrix<DerivedType, SubRows, SubCols>(
                static_cast<DerivedType &>(*this), row_start, col_start
            );
        }

        template <int SubRows, int SubCols>
        RefMatrix<const DerivedType, SubRows, SubCols>
        Submatrix(int row_start, int col_start) const {
            return RefMatrix<const DerivedType, SubRows, SubCols>(
                static_cast<const DerivedType &>(*this), row_start, col_start
            );
        }

        RefMatrix<DerivedType, 1, Cols> Row(int row_start) {
            return RefMatrix<DerivedType, 1, Cols>(
                static_cast<DerivedType &>(*this), row_start, 0
            );
        }

        RefMatrix<const DerivedType, 1, Cols> Row(int row_start) const {
            return RefMatrix<const DerivedType, 1, Cols>(
                static_cast<const DerivedType &>(*this), row_start, 0
            );
        }

        RefMatrix<DerivedType, Rows, 1> Column(int col_start) {
            return RefMatrix<DerivedType, Rows, 1>(
                static_cast<DerivedType &>(*this), 0, col_start
            );
        }

        RefMatrix<const DerivedType, Rows, 1> Column(int col_start) const {
            return RefMatrix<const DerivedType, Rows, 1>(
                static_cast<const DerivedType &>(*this), 0, col_start
            );
        }

        MatrixTranspose<DerivedType> operator~() {
            return MatrixTranspose<DerivedType>(
                static_cast<DerivedType &>(*this)
            );
        }

        MatrixTranspose<const DerivedType> operator~() const {
            return MatrixTranspose<const DerivedType>(
                static_cast<const DerivedType &>(*this)
            );
        }

        Matrix<Rows, Cols, DType> operator-() const {
            Matrix<Rows, Cols, DType> ret;

            for (int i = 0; i < rows; ++i) {
                for (int j = 0; j < cols; ++j) {
                    ret(i, j) = -(*this)(i, j);
                }
            }

            return ret;
        }

        size_t printTo(Print &p) const final {
            size_t n;
            n = p.print('[');

            for (int i = 0; i < Rows; i++) {
                n += p.print('[');

                for (int j = 0; j < Cols; j++) {
                    n += p.print(
                        static_cast<const DerivedType *>(this)->operator()(i, j)
                    );
                    n += p.print((j == Cols - 1) ? ']' : ',');
                }

                n += p.print((i == Rows - 1) ? ']' : ',');
            }
            return n;
        }
    };

    template <typename DerivedType>
    using DownCast = MatrixBase<
        DerivedType,
        DerivedType::Rows,
        DerivedType::Cols,
        typename DerivedType::DType>;

} // namespace BLA

namespace BLA {
    // This namespace exists because the header "typetraits" is not implemented
    // in every Arduino environment.
    namespace Types {
        template <class T, class U> struct is_same {
            static constexpr bool value = false;
        };
        template <class T> struct is_same<T, T> {
            static constexpr bool value = true;
        };

        template <class T> struct remove_const {
            typedef T type;
        };
        template <class T> struct remove_const<const T> {
            typedef T type;
        };

        template <class T> struct is_floating_point {
            static constexpr bool value =
                is_same<float, typename remove_const<T>::type>::value ||
                is_same<double, typename remove_const<T>::type>::value ||
                is_same<long double, typename remove_const<T>::type>::value;
        };

        template <class T> struct is_signed_integer {
            static constexpr bool value =
                is_same<signed char, typename remove_const<T>::type>::value ||
                is_same<signed short, typename remove_const<T>::type>::value ||
                is_same<signed int, typename remove_const<T>::type>::value ||
                is_same<signed long, typename remove_const<T>::type>::value ||
                is_same<signed long long, typename remove_const<T>::type>::
                    value;
        };

        template <class T> struct is_signed {
            static constexpr bool value =
                is_floating_point<T>::value || is_signed_integer<T>::value;
        };

        template <bool, typename T = void> struct enable_if {};
        template <typename T> struct enable_if<true, T> {
            typedef T type;
        };
    } // namespace Types
} // namespace BLA

namespace BLA {
    template <
        typename MatAType,
        typename MatBType,
        int MatARows,
        int MatACols,
        int MatBCols,
        typename DType>
    Matrix<MatARows, MatBCols, DType> operator*(
        const MatrixBase<MatAType, MatARows, MatACols, DType> &matA,
        const MatrixBase<MatBType, MatACols, MatBCols, DType> &matB
    ) {
        Matrix<MatARows, MatBCols, DType> ret;

        for (int i = 0; i < MatARows; ++i) {
            for (int j = 0; j < MatBCols; ++j) {
                if (MatACols > 0) {
                    ret(i, j) = matA(i, 0) * matB(0, j);
                }

                for (int k = 1; k < MatACols; k++) {
                    ret(i, j) += matA(i, k) * matB(k, j);
                }
            }
        }
        return ret;
    }

    template <
        typename MatAType,
        typename MatBType,
        int MatARows,
        int MatACols,
        int MatBCols,
        typename DType>
    MatrixBase<MatAType, MatARows, MatACols, DType> &operator*=(
        MatrixBase<MatAType, MatARows, MatACols, DType> &matA,
        const MatrixBase<MatBType, MatACols, MatBCols, DType> &matB
    ) {
        matA = matA * matB;
        return matA;
    }

    template <
        typename MatAType,
        typename MatBType,
        int Rows,
        int Cols,
        typename DType>
    MatrixBase<MatAType, Rows, Cols, DType> &operator+=(
        MatrixBase<MatAType, Rows, Cols, DType> &matA,
        const MatrixBase<MatBType, Rows, Cols, DType> &matB
    ) {
        for (int i = 0; i < Rows; ++i) {
            for (int j = 0; j < Cols; ++j) {
                matA(i, j) += matB(i, j);
            }
        }

        return matA;
    }

    template <
        typename MatAType,
        typename MatBType,
        int Rows,
        int Cols,
        typename DType>
    MatrixBase<MatAType, Rows, Cols, DType> &operator-=(
        MatrixBase<MatAType, Rows, Cols, DType> &matA,
        const MatrixBase<MatBType, Rows, Cols, DType> &matB
    ) {
        for (int i = 0; i < Rows; ++i) {
            for (int j = 0; j < Cols; ++j) {
                matA(i, j) -= matB(i, j);
            }
        }

        return matA;
    }

    template <typename MatType, int Rows, int Cols, typename DType>
    MatrixBase<MatType, Rows, Cols, DType> &
    operator*=(MatrixBase<MatType, Rows, Cols, DType> &mat, const DType k) {
        for (int i = 0; i < Rows; ++i) {
            for (int j = 0; j < Cols; ++j) {
                mat(i, j) *= k;
            }
        }
        return mat;
    }

    template <typename MatType, int Rows, int Cols, typename DType>
    MatrixBase<MatType, Rows, Cols, DType> &
    operator/=(MatrixBase<MatType, Rows, Cols, DType> &mat, const DType k) {
        for (int i = 0; i < Rows; ++i) {
            for (int j = 0; j < Cols; ++j) {
                mat(i, j) /= k;
            }
        }
        return mat;
    }
    template <typename MatType, int Rows, int Cols, typename DType>
    MatrixBase<MatType, Rows, Cols, DType> &
    operator+=(MatrixBase<MatType, Rows, Cols, DType> &mat, const DType k) {
        for (int i = 0; i < Rows; ++i) {
            for (int j = 0; j < Cols; ++j) {
                mat(i, j) += k;
            }
        }
        return mat;
    }
    template <typename MatType, int Rows, int Cols, typename DType>
    MatrixBase<MatType, Rows, Cols, DType> &
    operator-=(MatrixBase<MatType, Rows, Cols, DType> &mat, const DType k) {
        for (int i = 0; i < Rows; ++i) {
            for (int j = 0; j < Cols; ++j) {
                mat(i, j) -= k;
            }
        }
        return mat;
    }

    template <
        typename MatAType,
        typename MatBType,
        int Rows,
        int Cols,
        typename DType>
    Matrix<Rows, Cols, DType> operator+(
        const MatrixBase<MatAType, Rows, Cols, DType> &matA,
        const MatrixBase<MatBType, Rows, Cols, DType> &matB
    ) {
        Matrix<Rows, Cols, DType> ret = matA;
        ret += matB;
        return ret;
    }

    template <
        typename MatAType,
        typename MatBType,
        int Rows,
        int Cols,
        typename DType>
    Matrix<Rows, Cols, DType> operator-(
        const MatrixBase<MatAType, Rows, Cols, DType> &matA,
        const MatrixBase<MatBType, Rows, Cols, DType> &matB
    ) {
        Matrix<Rows, Cols, DType> ret = matA;
        ret -= matB;
        return ret;
    }

    template <typename MatType, int Rows, int Cols, typename DType>
    Matrix<Rows, Cols, DType> operator+(
        const MatrixBase<MatType, Rows, Cols, DType> &mat, const DType k
    ) {
        Matrix<Rows, Cols, DType> ret = mat;
        ret += k;
        return ret;
    }

    template <typename MatType, int Rows, int Cols, typename DType>
    Matrix<Rows, Cols, DType> operator+(
        const DType k, const MatrixBase<MatType, Rows, Cols, DType> &mat
    ) {
        return mat + k;
    }

    template <typename MatType, int Rows, int Cols, typename DType>
    Matrix<Rows, Cols, DType> operator-(
        const MatrixBase<MatType, Rows, Cols, DType> &mat, const DType k
    ) {
        Matrix<Rows, Cols, DType> ret = mat;
        ret -= k;
        return ret;
    }

    template <typename MatType, int Rows, int Cols, typename DType>
    Matrix<Rows, Cols, DType> operator-(
        const DType k, const MatrixBase<MatType, Rows, Cols, DType> &mat
    ) {
        return (-mat) + k;
    }

    template <typename MatType, int Rows, int Cols, typename DType>
    Matrix<Rows, Cols, DType> operator*(
        const MatrixBase<MatType, Rows, Cols, DType> &mat, const DType k
    ) {
        Matrix<Rows, Cols, DType> ret = mat;
        ret *= k;
        return ret;
    }

    template <typename MatType, int Rows, int Cols, typename DType>
    Matrix<Rows, Cols, DType> operator*(
        const DType k, const MatrixBase<MatType, Rows, Cols, DType> &mat
    ) {
        return mat * k;
    }

    template <typename MatType, int Rows, int Cols, typename DType>
    Matrix<Rows, Cols, DType> operator/(
        const MatrixBase<MatType, Rows, Cols, DType> &mat, const DType k
    ) {
        Matrix<Rows, Cols, DType> ret = mat;
        ret /= k;
        return ret;
    }

    template <typename MatType, int Rows, int Cols, typename DType>
    Matrix<Rows, Cols, DType> operator/(
        const DType k, const MatrixBase<MatType, Rows, Cols, DType> &mat
    ) {
        Matrix<Rows, Cols, DType> ret;
        for (int i = 0; i < Rows; ++i) {
            for (int j = 0; j < Cols; ++j) {
                ret(i, j) = k / mat(i, j);
            }
        }
        return ret;
    }

    template <
        typename DerivedType,
        typename OperandType,
        int Rows,
        typename DType>
    HorizontalConcat<DerivedType, OperandType> operator||(
        const MatrixBase<DerivedType, Rows, DerivedType::Cols, DType> &left,
        const MatrixBase<OperandType, Rows, OperandType::Cols, DType> &right
    ) {
        return HorizontalConcat<DerivedType, OperandType>(
            static_cast<const DerivedType &>(left),
            static_cast<const OperandType &>(right)
        );
    }

    template <
        typename DerivedType,
        typename OperandType,
        int Cols,
        typename DType>
    VerticalConcat<DerivedType, OperandType> operator&&(
        const MatrixBase<DerivedType, DerivedType::Rows, Cols, DType> &top,
        const MatrixBase<OperandType, OperandType::Rows, Cols, DType> &bottom
    )

    {
        return VerticalConcat<DerivedType, OperandType>(
            static_cast<const DerivedType &>(top),
            static_cast<const OperandType &>(bottom)
        );
    }

    template <
        typename MatAType,
        typename MatBType,
        int Rows,
        int Cols,
        typename DType>
    Matrix<Rows, Cols, bool> operator==(
        const MatrixBase<MatAType, Rows, Cols, DType> &matA,
        const MatrixBase<MatBType, Rows, Cols, DType> &matB
    ) {
        Matrix<Rows, Cols, bool> ret;

        for (int i = 0; i < Rows; ++i) {
            for (int j = 0; j < Cols; ++j) {
                ret(i, j) = matA(i, j) == matB(i, j);
            }
        }
        return ret;
    }

    template <typename MatAType, typename MatBType, int Rows, int Cols>
    Matrix<Rows, Cols, bool> operator&(
        const MatrixBase<MatAType, Rows, Cols, bool> &matA,
        const MatrixBase<MatBType, Rows, Cols, bool> &matB
    ) {
        Matrix<Rows, Cols, bool> ret;

        for (int i = 0; i < Rows; ++i) {
            for (int j = 0; j < Cols; ++j) {
                ret(i, j) = matA(i, j) & matB(i, j);
            }
        }
        return ret;
    }

    template <typename MatAType, typename MatBType, int Rows, int Cols>
    Matrix<Rows, Cols, bool> operator|(
        const MatrixBase<MatAType, Rows, Cols, bool> &matA,
        const MatrixBase<MatBType, Rows, Cols, bool> &matB
    ) {
        Matrix<Rows, Cols, bool> ret;

        for (int i = 0; i < Rows; ++i) {
            for (int j = 0; j < Cols; ++j) {
                ret(i, j) = matA(i, j) | matB(i, j);
            }
        }
        return ret;
    }

    template <typename DerivedType>
    Matrix<DerivedType::Rows, DerivedType::Cols, bool>
    operator!(const MatrixBase<
              DerivedType,
              DerivedType::Rows,
              DerivedType::Cols,
              bool> &matA) {
        Matrix<DerivedType::Rows, DerivedType::Cols, bool> ret;

        for (int i = 0; i < DerivedType::Rows; ++i) {
            for (int j = 0; j < DerivedType::Cols; ++j) {
                ret(i, j) = !matA(i, j);
            }
        }
        return ret;
    }

    template <
        typename MatAType,
        typename MatBType,
        int Rows,
        int Cols,
        typename DType>
    Matrix<Rows, Cols, bool> operator>(
        const MatrixBase<MatAType, Rows, Cols, DType> &matA,
        const MatrixBase<MatBType, Rows, Cols, DType> &matB
    ) {
        Matrix<Rows, Cols, bool> ret;

        for (int i = 0; i < Rows; ++i) {
            for (int j = 0; j < Cols; ++j) {
                ret(i, j) = matA(i, j) > matB(i, j);
            }
        }
        return ret;
    }

    template <
        typename MatAType,
        typename MatBType,
        int Rows,
        int Cols,
        typename DType>
    Matrix<Rows, Cols, bool> operator<(
        const MatrixBase<MatAType, Rows, Cols, DType> &matA,
        const MatrixBase<MatBType, Rows, Cols, DType> &matB
    ) {
        Matrix<Rows, Cols, bool> ret;

        for (int i = 0; i < Rows; ++i) {
            for (int j = 0; j < Cols; ++j) {
                ret(i, j) = matA(i, j) < matB(i, j);
            }
        }
        return ret;
    }

    template <
        typename MatAType,
        typename MatBType,
        int Rows,
        int Cols,
        typename DType>
    Matrix<Rows, Cols, bool> operator<=(
        const MatrixBase<MatAType, Rows, Cols, DType> &matA,
        const MatrixBase<MatBType, Rows, Cols, DType> &matB
    ) {
        Matrix<Rows, Cols, bool> ret;

        for (int i = 0; i < Rows; ++i) {
            for (int j = 0; j < Cols; ++j) {
                ret(i, j) = matA(i, j) <= matB(i, j);
            }
        }
        return ret;
    }

    template <
        typename MatAType,
        typename MatBType,
        int Rows,
        int Cols,
        typename DType>
    Matrix<Rows, Cols, bool> operator>=(
        const MatrixBase<MatAType, Rows, Cols, DType> &matA,
        const MatrixBase<MatBType, Rows, Cols, DType> &matB
    ) {
        Matrix<Rows, Cols, bool> ret;

        for (int i = 0; i < Rows; ++i) {
            for (int j = 0; j < Cols; ++j) {
                ret(i, j) = matA(i, j) >= matB(i, j);
            }
        }
        return ret;
    }

    template <typename DerivedType>
    Matrix<DerivedType::Rows, DerivedType::Cols, bool> operator>(
        const DownCast<DerivedType> &mat, const typename DerivedType::DType k
    ) {
        Matrix<DerivedType::Rows, DerivedType::Cols, bool> ret;

        for (int i = 0; i < DerivedType::Rows; ++i) {
            for (int j = 0; j < DerivedType::Cols; ++j) {
                ret(i, j) = mat(i, j) > k;
            }
        }
        return ret;
    }

    template <typename DerivedType>
    Matrix<DerivedType::Rows, DerivedType::Cols, bool> operator>=(
        const DownCast<DerivedType> &mat, const typename DerivedType::DType k
    ) {
        Matrix<DerivedType::Rows, DerivedType::Cols, bool> ret;

        for (int i = 0; i < DerivedType::Rows; ++i) {
            for (int j = 0; j < DerivedType::Cols; ++j) {
                ret(i, j) = mat(i, j) >= k;
            }
        }
        return ret;
    }

    template <typename DerivedType>
    Matrix<DerivedType::Rows, DerivedType::Cols, bool> operator<(
        const DownCast<DerivedType> &mat, const typename DerivedType::DType k
    ) {
        Matrix<DerivedType::Rows, DerivedType::Cols, bool> ret;

        for (int i = 0; i < DerivedType::Rows; ++i) {
            for (int j = 0; j < DerivedType::Cols; ++j) {
                ret(i, j) = mat(i, j) < k;
            }
        }
        return ret;
    }

    template <typename DerivedType>
    Matrix<DerivedType::Rows, DerivedType::Cols, bool> operator<=(
        const DownCast<DerivedType> &mat, const typename DerivedType::DType k
    ) {
        Matrix<DerivedType::Rows, DerivedType::Cols, bool> ret;

        for (int i = 0; i < DerivedType::Rows; ++i) {
            for (int j = 0; j < DerivedType::Cols; ++j) {
                ret(i, j) = mat(i, j) <= k;
            }
        }
        return ret;
    }

    template <typename DerivedType>
    bool
    Any(const MatrixBase<
        DerivedType,
        DerivedType::Rows,
        DerivedType::Cols,
        bool> &matA) {
        for (int i = 0; i < DerivedType::Rows; ++i) {
            for (int j = 0; j < DerivedType::Cols; ++j) {
                if (matA(i, j)) {
                    return true;
                }
            }
        }

        return false;
    }

    template <typename DerivedType>
    bool
    All(const MatrixBase<
        DerivedType,
        DerivedType::Rows,
        DerivedType::Cols,
        bool> &matA) {
        for (int i = 0; i < DerivedType::Rows; ++i) {
            for (int j = 0; j < DerivedType::Cols; ++j) {
                if (!matA(i, j)) {
                    return false;
                }
            }
        }

        return true;
    }

} // namespace BLA

namespace BLA {
    template <typename ParentType, typename Dtype>
    void Swap(
        MatrixBase<ParentType, ParentType::Rows, ParentType::Cols, Dtype> &A,
        MatrixBase<ParentType, ParentType::Rows, ParentType::Cols, Dtype> &B
    ) {
        Dtype tmp;
        for (int i = 0; i < ParentType::Rows; i++) {
            for (int j = 0; j < ParentType::Cols; j++) {
                tmp = A(i, j);
                A(i, j) = B(i, j);
                B(i, j) = tmp;
            }
        }
    }

    template <typename ParentTypeA, typename ParentTypeB, int Cols>
    Matrix<3, Cols, typename ParentTypeA::DType> CrossProduct(
        const MatrixBase<ParentTypeA, 3, Cols, typename ParentTypeA::DType>
            &matA,
        const MatrixBase<ParentTypeB, 3, Cols, typename ParentTypeA::DType>
            &matB
    ) {
        Matrix<3, Cols, typename ParentTypeA::DType> ret;
        for (int i = 0; i < Cols; ++i) {
            ret(0, i) = matA(1, i) * matB(2, i) - matB(1, i) * matA(2, i);
            ret(1, i) = matA(2, i) * matB(0, i) - matB(2, i) * matA(0, i);
            ret(2, i) = matA(0, i) * matB(1, i) - matB(0, i) * matA(1, i);
        }
        return ret;
    }

    template <typename ParentTypeA, typename ParentTypeB, int Dim>
    typename ParentTypeA::DType DotProduct(
        const MatrixBase<ParentTypeA, Dim, 1, typename ParentTypeA::DType>
            &vecA,
        const MatrixBase<ParentTypeB, Dim, 1, typename ParentTypeA::DType> &vecB
    ) {
        typename ParentTypeA::DType ret = 0;
        for (int i = 0; i < Dim; i++) {
            ret += vecA(i) * vecB(i);
        }
        return ret;
    }

    template <typename ParentType> struct LUDecomposition {
        bool singular;
        typename ParentType::DType parity;
        PermutationMatrix<ParentType::Rows, typename ParentType::DType> P;
        LowerUnitriangularMatrix<ParentType> L;
        UpperTriangularMatrix<ParentType> U;

        LUDecomposition(
            MatrixBase<
                ParentType,
                ParentType::Rows,
                ParentType::Cols,
                typename ParentType::DType> &A
        )
            : L(static_cast<ParentType &>(A)), U(static_cast<ParentType &>(A)) {
            static_assert(
                ParentType::Rows == ParentType::Cols,
                "Input matrix must be square"
            );
        }
    };

    template <typename ParentType> struct CholeskyDecomposition {
        bool positive_definite = true;
        LowerTriangularMatrix<ParentType> L;

        CholeskyDecomposition(
            MatrixBase<
                ParentType,
                ParentType::Rows,
                ParentType::Cols,
                typename ParentType::DType> &A
        )
            : L(static_cast<ParentType &>(A)) {}
    };

    template <typename ParentType, int Dim>
    LUDecomposition<ParentType> LUDecompose(
        MatrixBase<ParentType, Dim, Dim, typename ParentType::DType> &A
    ) {
        LUDecomposition<ParentType> decomp(A);
        auto &idx = decomp.P.idx;
        decomp.parity = 1.0;

        for (int i = 0; i < Dim; ++i) {
            idx[i] = i;
        }

        // row_scale stores the implicit scaling of each row
        typename ParentType::DType row_scale[Dim];

        for (int i = 0; i < Dim; ++i) {
            // Loop over rows to get the implicit scaling information.
            typename ParentType::DType largest_elem = 0.0;

            for (int j = 0; j < Dim; ++j) {
                typename ParentType::DType this_elem = fabs(A(i, j));
                largest_elem = max(this_elem, largest_elem);
            }

            // No nonzero largest element.
            if (largest_elem == 0.0) {
                decomp.singular = true;
                return decomp;
            }

            row_scale[i] = 1.0 / largest_elem;
        }

        // This is the loop over columns of Crout’s method.
        for (int j = 0; j < Dim; ++j) {
            // Calculate beta ij
            for (int i = 0; i < j; ++i) {
                typename ParentType::DType sum = 0.0;

                for (int k = 0; k < i; ++k) {
                    sum += A(i, k) * A(k, j);
                }

                A(i, j) -= sum;
            }

            // Calcuate alpha ij (before division by the pivot)
            for (int i = j; i < Dim; ++i) {
                typename ParentType::DType sum = 0.0;

                for (int k = 0; k < j; ++k) {
                    sum += A(i, k) * A(k, j);
                }

                A(i, j) -= sum;
            }

            // Search for largest pivot element
            typename ParentType::DType largest_elem = 0.0;
            int argmax = j;

            for (int i = j; i < Dim; i++) {
                typename ParentType::DType this_elem =
                    row_scale[i] * fabs(A(i, j));

                if (this_elem >= largest_elem) {
                    largest_elem = this_elem;
                    argmax = i;
                }
            }

            if (j != argmax) {
                auto row_argmax = A.Row(argmax);
                auto row_j = A.Row(j);
                Swap(row_argmax, row_j);

                decomp.parity = -decomp.parity;

                // swap indices
                {
                    auto tmp = idx[j];
                    idx[j] = idx[argmax];
                    idx[argmax] = tmp;
                }

                row_scale[argmax] = row_scale[j];
            }

            if (A(j, j) == 0.0) {
                decomp.singular = true;
                return decomp;
            }

            if (j != Dim) {
                // Now, finally, divide by the pivot element.
                typename ParentType::DType pivot_inv = 1.0 / A(j, j);

                for (int i = j + 1; i < Dim; ++i) {
                    A(i, j) *= pivot_inv;
                }
            }
        }

        decomp.singular = false;
        return decomp;
    }

    template <int Dim, class LUType, class BType>
    Matrix<Dim, 1, typename BType::DType> LUSolve(
        const LUDecomposition<LUType> &decomp,
        const MatrixBase<BType, Dim, 1, typename BType::DType> &b
    ) {
        Matrix<Dim, 1, typename BType::DType> x, tmp;

        auto &idx = decomp.P.idx;
        auto &LU = decomp.L.parent;

        // Forward substitution to solve L * y = b
        for (int i = 0; i < Dim; ++i) {
            typename BType::DType sum = 0.0;

            for (int j = 0; j < i; ++j) {
                sum += LU(i, j) * tmp(idx[j]);
            }

            tmp(idx[i]) = b(idx[i]) - sum;
        }

        // Backward substitution to solve U * x = y
        for (int i = Dim - 1; i >= 0; --i) {
            typename BType::DType sum = 0.0;

            for (int j = i + 1; j < Dim; ++j) {
                sum += LU(i, j) * tmp(idx[j]);
            }

            tmp(idx[i]) = (tmp(idx[i]) - sum) / LU(i, i);
        }

        // Undo the permutation
        for (int i = 0; i < Dim; ++i) {
            x(i) = tmp(idx[i]);
        }

        return x;
    }

    template <typename ParentType, int Dim>
    CholeskyDecomposition<ParentType> CholeskyDecompose(
        MatrixBase<ParentType, Dim, Dim, typename ParentType::DType> &A
    ) {
        CholeskyDecomposition<ParentType> chol(A);

        for (int i = 0; i < Dim; ++i) {
            for (int j = i; j < Dim; ++j) {
                float sum = A(i, j);

                for (int k = i - 1; k >= 0; --k) {
                    sum -= A(i, k) * A(j, k);
                }

                if (i == j) {
                    if (sum <= 0.0) {
                        chol.positive_definite = false;
                        return chol;
                    }
                    A(i, i) = sqrt(sum);
                } else {
                    A(j, i) = sum / A(i, i);
                }
            }
        }

        return chol;
    }

    template <int Dim, class LUType, class BType>
    Matrix<Dim, 1, typename BType::DType> CholeskySolve(
        const CholeskyDecomposition<LUType> &decomp,
        const MatrixBase<BType, Dim, 1, typename BType::DType> &b
    ) {
        Matrix<Dim, 1, typename BType::DType> x;
        auto &A = decomp.L.parent;

        for (int i = 0; i < Dim; ++i) {
            float sum = b(i);

            for (int k = i - 1; k >= 0; --k) {
                sum -= A(i, k) * x(k);
            }

            x(i) = sum / A(i, i);
        }

        for (int i = Dim - 1; i >= 0; --i) {
            float sum = x(i);

            for (int k = i + 1; k < Dim; ++k) {
                sum -= A(k, i) * x(k);
            }

            x(i) = sum / A(i, i);
        }

        return x;
    }

    template <int Dim, typename InType, typename OutType, typename DType>
    bool Invert(
        const MatrixBase<InType, Dim, Dim, DType> &A,
        MatrixBase<OutType, Dim, Dim, DType> &out
    ) {
        Matrix<Dim, Dim, DType> A_copy = A;

        auto decomp = LUDecompose(A_copy);

        if (decomp.singular) {
            return false;
        }

        Matrix<Dim, 1, DType> b = Zeros<Dim, 1, DType>();

        for (int j = 0; j < Dim; ++j) {
            b(j) = 1.0;
            out.Column(j) = LUSolve(decomp, b);
            b(j) = 0.0;
        }

        return true;
    }

    template <int Dim, class ParentType>
    bool
    Invert(MatrixBase<ParentType, Dim, Dim, typename ParentType::DType> &A) {
        return Invert(A, A);
    }

    template <int Dim, class ParentType>
    Matrix<Dim, Dim, typename ParentType::DType> Inverse(
        const MatrixBase<ParentType, Dim, Dim, typename ParentType::DType> &A
    ) {
        Matrix<Dim, Dim, typename ParentType::DType> out;
        Invert(A, out);
        return out;
    }

    // LU-Decomposition only works for floating point numbers. Use Bareiss
    // algorithm for (signed) integer types.
    template <typename ParentType, typename Dtype, int Dim>
    typename Types::enable_if<Types::is_floating_point<Dtype>::value, Dtype>::
        type
        DeterminantLUDecomposition(
            const MatrixBase<ParentType, Dim, Dim, Dtype> &A
        ) {
        Matrix<Dim, Dim, Dtype> A_copy = A;

        auto decomp = LUDecompose(A_copy);

        Dtype det = decomp.parity;

        for (int i = 0; i < Dim; ++i) {
            det *= decomp.U(i, i);
        }

        return det;
    }

    // Bareiss algorithm works for all (signed) types, but for floating-point
    // numbers LU-Decomposition is faster.
    template <typename ParentType, typename Dtype, int Dim>
    typename Types::enable_if<Types::is_signed<Dtype>::value, Dtype>::type
    DeterminantBareissAlgorithm(
        const MatrixBase<ParentType, Dim, Dim, Dtype> &A
    ) {
        Matrix<Dim, Dim, Dtype> A_copy = A;

        int sign = 1;
        Dtype prev = 1;

        for (int i = 0; i < Dim; i++) {
            if (A_copy(i, i) == 0) {
                int j = i + 1;
                for (; j < Dim; j++) {
                    if (A_copy(j, i) != 0)
                        break;
                }
                if (j == Dim)
                    return 0;
                auto row_i = A_copy.Row(i);
                auto row_j = A_copy.Row(j);
                Swap(row_i, row_j);
                sign = -sign;
            }
            for (int j = i + 1; j < Dim; j++) {
                for (int k = i + 1; k < Dim; k++) {
                    A_copy(j, k) = (A_copy(j, k) * A_copy(i, i) -
                                    A_copy(j, i) * A_copy(i, k)) /
                                   prev;
                }
            }
            prev = A_copy(i, i);
        }
        return sign * A_copy(Dim - 1, Dim - 1);
    }

    template <typename ParentType, typename Dtype, int Dim>
    typename Types::enable_if<Types::is_floating_point<Dtype>::value, Dtype>::
        type
        Determinant(const MatrixBase<ParentType, Dim, Dim, Dtype> &A) {
        return DeterminantLUDecomposition(A);
    }

    template <typename ParentType, typename Dtype, int Dim>
    typename Types::enable_if<Types::is_signed_integer<Dtype>::value, Dtype>::
        type
        Determinant(const MatrixBase<ParentType, Dim, Dim, Dtype> &A) {
        return DeterminantBareissAlgorithm(A);
    }

    template <typename DerivedType>
    typename DerivedType::DType Norm(const DownCast<DerivedType> &A) {
        typename DerivedType::DType sum_sq = 0.0;

        for (int i = 0; i < DerivedType::Rows; ++i) {
            for (int j = 0; j < DerivedType::Cols; ++j) {
                sum_sq += A(i, j) * A(i, j);
            }
        }
        return sqrt(sum_sq);
    }

    template <class DerivedType>
    typename DerivedType::DType Trace(const DownCast<DerivedType> &A) {
        typename DerivedType::DType sum_diag = 0.0;

        for (int i = 0; i < DerivedType::Rows; ++i) {
            sum_diag += A(i, i);
        }
        return sum_diag;
    }

    template <int Inputs, int Outputs, typename DType> struct MatrixFunctor {
        virtual Matrix<Outputs, 1, DType>
        operator()(const Matrix<Inputs, 1, DType> &x) const = 0;
    };

    template <int Inputs, int Outputs, typename InType>
    Matrix<Outputs, Inputs, typename InType::DType> Jacobian(
        const MatrixFunctor<Inputs, Outputs, typename InType::DType> &f,
        const MatrixBase<InType, Inputs, 1, typename InType::DType> &x,
        const typename InType::DType h = 1e-4
    ) {
        using DType = typename InType::DType;

        Matrix<Outputs, Inputs, DType> jacobian;
        Matrix<Outputs> f_x = f(x);
        Matrix<Inputs, 1, DType> h_vec = Zeros<Inputs, 1, DType>();

        for (int i = 0; i < Inputs; i++) {
            h_vec(i) = h;
            Matrix<Outputs, 1, DType> f_xh = f(x + h_vec);
            jacobian.Column(i) = (f_xh - f_x) / h;
            h_vec(i) = 0;
        }

        return jacobian;
    }

} // namespace BLA
