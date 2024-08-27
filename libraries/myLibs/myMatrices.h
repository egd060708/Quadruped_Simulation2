#pragma once

#include <iostream>
#include <vector>
#include <stdexcept>

#define USING_NAMESPACE_MM using namespace mM;
#define MATRIX myMatrices<real_t>
#define MATRIX_PTR myMatrices<real_t>*

namespace mM {

    template <class T>
    class myMatrices {
    /* ��Ԫ���� */
        // template<class T>
        // friend myMatrices<T> rowCombine(myMatrices<T>& A, myMatrices<T>& B);
        // template<class T, class... Args>
        // friend myMatrices<T> rowCombine(myMatrices<T>& A, myMatrices<T>& B, Args... rest);
        // template<class T>
        // friend myMatrices<T> colCombine(myMatrices<T>& A, myMatrices<T>& B);
        // template<class T, class... Args>
        // friend myMatrices<T> colCombine(myMatrices<T>& A, myMatrices<T>& B, Args... rest);

    private:
        uint16_t rows;
        uint16_t cols;
        T* data;

    public:
        /* ���й��� */
        myMatrices(uint16_t rows, uint16_t cols) : rows(rows), cols(cols)
        {
            data = new T[rows * cols];
            clear(0);
        }
        /* ������ */
        myMatrices(uint16_t n) : rows(n), cols(n)
        {
            data = new T[rows * cols];
            clear(0);
        }

        /* ������������ֵͳһ�� */
        void clear(T num)
        {
            if (rows > 0 && cols > 0)
            {
                for (uint16_t i = 0; i < rows; i++)
                {
                    for (uint16_t j = 0; j < cols; j++)
                    {
                        data[i * cols + j] = num;
                    }
                }
            }
        }

        /* ��λ���� */
        void eye()
        {
            if ((rows > 0) && (cols > 0) && (rows == cols))
            {
                for (uint16_t i = 0; i < rows; i++)
                {
                    data[i * cols + i] = 1;
                }
            }
        }
        
        /* ��ȡ��������� */
        uint16_t getRows() const {
            return rows;
        }

        /* ��ȡ��������� */
        uint16_t getCols() const {
            return cols;
        }
        
        /* ��ȡ�������ض�λ�õ�Ԫ�� */
        T getElement(uint16_t row, uint16_t col) const {
            return data[row * cols + col];
        }
        
        /* ���þ������ض�λ�õ�Ԫ�� */
        void setElement(uint16_t row, uint16_t col, T value) {
            data[row * cols + col] = value;
        }

        /* ���������������� */
        void setArray(const T* array, uint16_t length)
        {
            if (rows == length / cols)
            {
                memcpy(data, array, rows * cols * sizeof(T));
            }
            else
            {
            }
        }
        
        /* ���������� */
        void setRowArray(const T* _array, uint16_t _row)
        {
            if (_row < rows)
            {
                for (int i = 0; i < cols; i++)
                {
                    data[cols * _row + i] = _array[i];//ȡ����Ӧ�����ϵ�ÿһ��Ԫ��
                }
            }
        }
        
        /* ���������� */
        void setColArray(const T* _array, uint16_t _col)
        {
            if (_col < cols)
            {
                for (int i = 0; i < rows; i++)
                {
                    data[i * cols + _col] = _array[i];//ȡ����Ӧ�����ϵ�ÿһ��Ԫ��
                }
            }
        }
        
        /* ����Ĭ���������͵��������� */
        const T* getArray() {
            return data;
        }
        
        /* ���������� */
        void getRowArray(T* _dst, uint16_t _row)
        {
            if (_row < rows)
            {
                for (int i = 0; i < cols; i++)
                {
                    _dst[i] = data[cols * _row + i];//ȡ����Ӧ�����ϵ�ÿһ��Ԫ��
                }
            }
        }
        
        /* ���������� */
        void getColArray(T* _dst, uint16_t _col)
        {
            if (_col < cols)
            {
                for (int i = 0; i < rows; i++)
                {
                    _dst[i] = data[i * cols + _col];//ȡ����Ӧ�����ϵ�ÿһ��Ԫ��
                }
            }
        }
        
        /* ��ӡ���� */
        void print() const {
            for (uint16_t i = 0; i < rows; i++) {
                for (uint16_t j = 0; j < cols; j++) {
                    std::cout << getElement(i, j) << " ";
                }
                std::cout << std::endl;
            }
            std::cout << std::endl;
        }

        //����ӷ�
        myMatrices<T> operator+(const myMatrices<T>& other) const;
        // �������
        myMatrices<T> operator-(const myMatrices<T>& other) const;
        // ����˷�
        myMatrices<T> operator*(const myMatrices<T>& other) const;
        myMatrices<T> operator*(const T other) const;
        // ����������ʽ
        T determinant() const;
        // �������������ʽ
        myMatrices<T> getCofactorMatrix(uint16_t rowToRemove, uint16_t colToRemove) const;
        // ����ת��
        myMatrices<T> transpose() const;
        // ��������
        myMatrices<T> inverse() const;
        // ����������
        void rowExpansion(myMatrices<T> other);
        // ����������
        void colExpansion(myMatrices<T> other);
    };



    /* ����ӷ� */
    template<class T>
    myMatrices<T> myMatrices<T>::operator+(const myMatrices<T>& other) const {

        myMatrices<T> result(rows, cols);

        if (rows != other.rows || cols != other.cols) {
            throw std::invalid_argument("Matrix dimensions are not compatible for addition");
        }
        else
        {
            for (uint16_t i = 0; i < rows; i++) {
                for (uint16_t j = 0; j < cols; j++) {
                    result.setElement(i, j, getElement(i, j) + other.getElement(i, j));
                }
            }
        }

        return result;
    }

    /* ������� */
    template<class T>
    myMatrices<T> myMatrices<T>::operator-(const myMatrices<T>& other) const {

        myMatrices<T> result(rows, cols);

        if (rows != other.rows || cols != other.cols) {
            throw std::invalid_argument("Matrix dimensions are not compatible for subtraction");
            return result;
        }
        else
        {
            for (uint16_t i = 0; i < rows; i++) {
                for (uint16_t j = 0; j < cols; j++) {
                    result.setElement(i, j, getElement(i, j) - other.getElement(i, j));
                }
            }
        }

        return result;
    }

    /* ����˷� */
    template<class T>
    myMatrices<T> myMatrices<T>::operator*(const myMatrices<T>& other) const {

        myMatrices<T> result(rows, other.cols);

        if (cols != other.rows) {
            throw std::invalid_argument("Matrix dimensions are not compatible for multiplication");
        }
        else
        {
            for (uint16_t i = 0; i < rows; i++) {
                for (uint16_t j = 0; j < other.cols; j++) {
                    T sum = 0;
                    for (uint16_t k = 0; k < cols; k++) {
                        sum += getElement(i, k) * other.getElement(k, j);
                    }
                    result.setElement(i, j, sum);
                }
            }
        }

        return result;
    }

    /* �����볣����� */
    template<class T>
    myMatrices<T> myMatrices<T>::operator*(const T other) const {
        myMatrices<T> result(rows, cols);
        for (uint16_t i = 0; i < rows; i++)
        {
            for (uint16_t j = 0; j < cols; j++)
            {
                result.setElement(i, j, this->getElement(i, j) * other);
            }
        }
        return result;
    }
 
    /* ����ת�� */
    template<class T>
    myMatrices<T> myMatrices<T>::transpose() const {
        myMatrices<T> result(cols, rows);

        for (uint16_t i = 0; i < rows; i++) {
            for (uint16_t j = 0; j < cols; j++) {
                result.setElement(j, i, getElement(i, j));
            }
        }

        return result;
    }

    /* �������� */
    template<class T>
    myMatrices<T> myMatrices<T>::inverse() const {

        myMatrices<T> adj(rows, cols);

        if (rows != cols) {
            throw std::invalid_argument("Matrix is not square");
            return adj;
        }

        T det = determinant();
        if (det == 0) {
            throw std::runtime_error("Matrix is singular; cannot compute inverse");
            return adj;
        }

        for (uint16_t i = 0; i < rows; i++) {
            for (uint16_t j = 0; j < cols; j++) {
                // ����(i, j)λ�õĴ�������ʽ
                myMatrices<T> cofactorMatrix = getCofactorMatrix(i, j);
                double cofactor = cofactorMatrix.determinant();

                // ʹ�ð�������(i, j)λ�ô洢��������ʽ
                adj.setElement(i, j, cofactor);
            }
        }

        return adj * (1. / det);
    }

    /* ������������ʽ */
    template<class T>
    T myMatrices<T>::determinant() const {
        if (rows != cols) {
            throw std::invalid_argument("Matrix must be square to calculate determinant");
            return 0;
        }

        if (rows == 1) {
            return getElement(0, 0);
        }

        if (rows == 2) {
            // ����2x2��������ʽ���㹫ʽΪ ad - bc
            return getElement(0, 0) * getElement(1, 1) - getElement(0, 1) * getElement(1, 0);
        }

        double det = 0;
        for (uint16_t i = 0; i < cols; i++) {
            // �����������ʽ��ֵ
            double cofactor = getElement(0, i) * getCofactorMatrix(0, i).determinant();
            // ʹ�õݹ��������ʽ
            det += (i % 2 == 0 ? 1 : -1) * cofactor;
        }

        return det;
    }

    /* ������������ʽ */
    template<class T>
    myMatrices<T> myMatrices<T>::getCofactorMatrix(uint16_t rowToRemove, uint16_t colToRemove) const {

        myMatrices<T> cofactor(rows - 1, cols - 1);

        if (rowToRemove < 0 || rowToRemove >= rows || colToRemove < 0 || colToRemove >= cols) {
            throw std::invalid_argument("Invalid row or column index");
            return cofactor;
        }

        uint16_t cofactorRow = 0;
        for (uint16_t i = 0; i < rows; i++) {
            if (i == rowToRemove) {
                continue; // ����Ҫ�Ƴ�����
            }

            uint16_t cofactorCol = 0;
            for (uint16_t j = 0; j < cols; j++) {
                if (j == colToRemove) {
                    continue; // ����Ҫ�Ƴ�����
                }

                cofactor.setElement(cofactorRow, cofactorCol, getElement(i, j));
                cofactorCol++;
            }

            cofactorRow++;
        }

        return cofactor;
    }

    /* ���������� */
    template<class T>
    void myMatrices<T>::rowExpansion(myMatrices<T> other)
    {
        if (cols != other.cols){
            throw  std::invalid_argument("Matrix dimensions are not compatible for expansion");
        }
        else
        {
            T* p = data;//��ȡԭ���ڴ��ַ
            data = new T[(rows + other.rows) * cols];//���·����ڴ�
            memcpy(data, p, sizeof(T)*rows*cols);//��ԭ����copy
            memcpy(data+(rows * cols), other.getArray(), sizeof(T) * other.rows * other.cols);//copy��������
            rows = rows + other.rows;//��������
            delete p;//ɾ��ԭ�����ڴ�
        }
    }

    /* ���������� */
    template<class T>
    void myMatrices<T>::colExpansion(myMatrices<T> other)
    {
        if (rows != other.rows) {
            throw  std::invalid_argument("Matrix dimensions are not compatible for expansion");
        }
        else
        {
            T* p = data;//��ȡԭ���ڴ��ַ
            data = new T[rows * (cols + other.cols)];//���·����ڴ�
            uint16_t n = cols + other.cols;
            for (uint16_t i = 0; i < rows; i++)
            {
                memcpy(data + i*n, p + i*cols, sizeof(T) * cols);//��ԭ����copy
                memcpy(data + i * n + cols, other.getArray() + i * other.cols, sizeof(T) * other.cols);//copy��������
            }
            cols = cols + other.cols;//��������
            delete p;//ɾ��ԭ�����ڴ�  
        }
    }

    /* ���� Kronecker �˻����ο�matlab��kron������ */
    template <class T>
    myMatrices<T> kron(myMatrices<T>& A, myMatrices<T>& B) {
        uint16_t m = A.getRows();
        uint16_t n = A.getCols();
        uint16_t p = B.getRows();
        uint16_t q = B.getCols();

        myMatrices<T> result(m * p, n * q);

        for (uint16_t i = 0; i < m; i++) {
            for (uint16_t j = 0; j < n; j++) {
                for (uint16_t k = 0; k < p; k++) {
                    for (uint16_t l = 0; l < q; l++) {
                        result.setElement(i * p + k, j * q + l, A.getElement(i, j) * B.getElement(k, l));
                    }
                }
            }
        }

        return result;
    }

    template <class T, class... Args>
    myMatrices<T> kron(myMatrices<T>& A, myMatrices<T>& B, Args&... rest) {
        uint16_t m = A.getRows();
        uint16_t n = A.getCols();
        uint16_t p = B.getRows();
        uint16_t q = B.getCols();

        myMatrices<T> result(m * p, n * q);

        for (uint16_t i = 0; i < m; i++) {
            for (uint16_t j = 0; j < n; j++) {
                for (uint16_t k = 0; k < p; k++) {
                    for (uint16_t l = 0; l < q; l++) {
                        result.setElement(i * p + k, j * q + l, A.getElement(i, j) * B.getElement(k, l));
                    }
                }
            }
        }

        return kron(result, rest...);
    }

    /* �����Խǿ���� */
    template <class T>
    myMatrices<T> blkdiag(const myMatrices<T>& A, const myMatrices<T>& B) {

        myMatrices<T> result(A.getRows() + B.getRows(), A.getCols() + B.getCols());

        uint16_t rowOffset = 0;
        uint16_t colOffset = 0;

        for (uint16_t j = 0; j < A.getRows(); j++) {
            for (uint16_t k = 0; k < A.getCols(); k++) {
                result.setElement(rowOffset + j, colOffset + k, A.getElement(j, k));
            }
        }

        rowOffset += A.getRows();
        colOffset += A.getCols();

        for (uint16_t j = 0; j < B.getRows(); j++) {
            for (uint16_t k = 0; k < B.getCols(); k++) {
                result.setElement(rowOffset + j, colOffset + k, B.getElement(j, k));
            }
        }

        rowOffset += B.getRows();
        colOffset += B.getCols();

        return result;
    }

    template <class T, class... Args>
    myMatrices<T> blkdiag(const myMatrices<T>& A, const myMatrices<T>& B, const Args&... rest) {

        myMatrices<T> result(A.getRows() + B.getRows(), A.getCols() + B.getCols());

        uint16_t rowOffset = 0;
        uint16_t colOffset = 0;

        for (uint16_t j = 0; j < A.getRows(); j++) {
            for (uint16_t k = 0; k < A.getCols(); k++) {
                result.setElement(rowOffset + j, colOffset + k, A.getElement(j, k));
            }
        }

        rowOffset += A.getRows();
        colOffset += A.getCols();

        for (uint16_t j = 0; j < B.getRows(); ++j) {
            for (uint16_t k = 0; k < B.getCols(); ++k) {
                result.setElement(rowOffset + j, colOffset + k, B.getElement(j, k));
            }
        }

        rowOffset += B.getRows();
        colOffset += B.getCols();

        return blkdiag(result, rest...);
    }

    /* ��������� */
    template<class T>
    myMatrices<T> zeros(uint16_t n)
    {
        myMatrices<T> result(n, n);
        return result;
    }
    template<class T>
    myMatrices<T> zeros(uint16_t rows, uint16_t cols)
    {
        myMatrices<T> result(rows, cols);
        return result;
    }

    /* ������λ�Խ��� */
    template<class T>
    myMatrices<T> eye(uint16_t n)
    {
        myMatrices<T> result(n, n);
        for (uint16_t i = 0; i < n; i++)
        {
            result.setElement(i, i, 1);
        }
        return result;
    }

    /* ��������ϲ����в����б䣩 */
    template<class T>
    myMatrices<T> rowCombine(myMatrices<T>& A,myMatrices<T>& B)
    {
        myMatrices<T> result(A.getRows() + B.getRows(), A.getCols());
        if (A.getCols() != B.getCols()) {
            throw  std::invalid_argument("Matrix dimensions are not compatible for combine");
            return result;
        }
        else
        {
            memcpy(result.data, A.getArray(), sizeof(T) * A.getRows() * A.getCols());//��ԭ����copy
            memcpy(result.data + (A.getRows() * A.getCols()), B.getArray(), sizeof(T) * B.getRows() * B.getCols());//copy��������
            return result;
        }
    }

    template <class T, class... Args>
    myMatrices<T> rowCombine(myMatrices<T>& A, myMatrices<T>& B,Args... rest)
    {
        myMatrices<T> result(A.getRows() + B.getRows(), A.getCols());
        if (A.getCols() != B.getCols()) {
            throw  std::invalid_argument("Matrix dimensions are not compatible for combine");
            return result;
        }
        else
        {
            memcpy(result.data, A.getArray(), sizeof(T) * A.getRows() * A.getCols());//��ԭ����copy
            memcpy(result.data + (A.getRows() * A.getCols()), B.getArray(), sizeof(T) * B.getRows() * B.getCols());//copy��������
            return rowCombine(result,rest...);
        }
    }

    /* ��������ϲ����б��в��䣩 */
    template<class T>
    myMatrices<T> colCombine(myMatrices<T>& A, myMatrices<T>& B)
    {
        myMatrices<T> result(A.getRows(), A.getCols() + B.getCols());
        if (A.getRows() != B.getRows()) {
            throw  std::invalid_argument("Matrix dimensions are not compatible for combine");
            return result;
        }
        else
        {
            uint16_t n = A.getCols() + B.getCols();
            for (uint16_t i = 0; i < A.getRows(); i++)
            {
                memcpy(result.data + i * n, A.getArray() + i * A.getCols(), sizeof(T) * A.getCols());//��ԭ����copy
                memcpy(result.data + i * n + A.getCols(), B.getArray() + i * B.getCols(), sizeof(T) * B.getCols());//copy��������
            }
            return result;
        }
    }

    template<class T, class... Args>
    myMatrices<T> colCombine(myMatrices<T>& A, myMatrices<T>& B, Args... rest)
    {
        myMatrices<T> result(A.getRows(), A.getCols() + B.getCols());
        if (A.getRows() != B.getRows()) {
            throw  std::invalid_argument("Matrix dimensions are not compatible for combine");
            return result;
        }
        else
        {
            uint16_t n = A.getCols() + B.getCols();
            for (uint16_t i = 0; i < A.getRows(); i++)
            {
                memcpy(result.data + i * n, A.getArray() + i * A.getCols(), sizeof(T) * A.getCols());//��ԭ����copy
                memcpy(result.data + i * n + A.getCols(), B.getArray() + i * B.getCols(), sizeof(T) * B.getCols());//copy��������
            }
            return colCombine(result,rest...);
        }
    }

}
