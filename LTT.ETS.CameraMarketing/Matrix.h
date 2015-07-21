#pragma once
#include <iostream>
#include "Parameter.h"

using namespace std;

template <typename T>
class Matrix
{
public:
	Matrix();
	Matrix(int rows, int columns);
	Matrix(const Matrix<T> &other);
	Matrix<T> & operator= (const Matrix<T> &other);
	~Matrix();

	void resize(int rows, int columns);
	void identity(void);
	void clear(void);
	T& operator () (int x, int y);
	T trace(void);
	Matrix<T>& transpose(void);

	Matrix<T> product(Matrix<T> &other);

	int minsize(void)
	{
		return ((m_rows < m_columns) ? m_rows : m_columns);
	}

	int columns(void)
	{
		return m_columns;
	}

	int rows(void)
	{
		return m_rows;
	}
private:
	T **m_matrix;
	int m_rows;
	int m_columns;
};


#include "Matrix.h"
#include <cassert>
#include <cstdlib>
#include <algorithm>
using namespace std;

template <typename T>
Matrix<T>::Matrix(void)
{
	m_rows = 0;
	m_columns = 0;
	m_matrix = NULL;
}


template <typename T>
Matrix<T>::Matrix(int rows, int columns)
{
	m_matrix = NULL;
	resize(rows, columns);
}

template <typename T>
Matrix<T>::Matrix(const Matrix<T>& other)
{
	if ( other.m_matrix != NULL )
	{
		m_matrix = NULL;
		resize(other.m_rows, other.m_columns);

		for ( int i = 0 ; i < m_rows ; i++ )
			for ( int j = 0 ; j < m_columns ; j++ )
				m_matrix[i][j] = other.m_matrix[i][j];
	} 
	else
	{
		m_matrix = NULL;
		m_rows = 0;
		m_columns = 0;
	}
}

template <typename T>
void Matrix<T>::resize(int rows, int columns)
{
	if ( m_matrix == NULL ) 
	{
		m_matrix = new T*[rows];
		for ( int i = 0 ; i < rows ; i++ )
			m_matrix[i] = new T[columns];

		m_rows = rows;
		m_columns = columns;
		clear();
	} 
	else 
	{
		T **new_matrix;
		new_matrix = new T*[rows]; 
		for ( int i = 0 ; i < rows ; i++ )
		{
			new_matrix[i] = new T[columns]; 
			for ( int j = 0 ; j < columns ; j++ )
				new_matrix[i][j] = 0;
		}

		int minrows = std::min<int>(rows, m_rows);
		int mincols = std::min<int>(columns, m_columns);

		for ( int x = 0 ; x < minrows ; x++ )
			for ( int y = 0 ; y < mincols ; y++ )
				new_matrix[x][y] = m_matrix[x][y];

		if ( m_matrix != NULL )
		{
			for ( int i = 0 ; i < m_rows ; i++ )
				delete [] m_matrix[i];

			delete [] m_matrix;
			m_matrix = NULL;
		}

		m_matrix = new_matrix;
	}

	m_rows = rows;
	m_columns = columns;
}

template <typename T>
void Matrix<T>::identity(void)
{
	assert( m_matrix != NULL );
	clear();

	int x = std::min<int>(m_rows, m_columns);
	for ( int i = 0 ; i < x ; i++ )
		m_matrix[i][i] = 1;
}

template <typename T>
void Matrix<T>::clear()
{
	assert( m_matrix != NULL );
	for ( int i = 0 ; i < m_rows ; i++ )
		for ( int j = 0 ; j < m_columns ; j++ )
			m_matrix[i][j] = 0;
}

template <typename T>
Matrix<T>& Matrix<T>::operator=(const Matrix<T>& other)
{
	if ( other.m_matrix != NULL ) 
	{
		resize(other.m_rows, other.m_columns);
		for ( int i = 0 ; i < m_rows ; i++ )
			for ( int j = 0 ; j < m_columns ; j++ )
				m_matrix[i][j] = other.m_matrix[i][j];
	} 
	else 
	{
		for ( int i = 0 ; i < m_columns ; i++ )
			delete [] m_matrix[i];

		delete [] m_matrix;
		m_matrix = NULL;
		m_rows = 0;
		m_columns = 0;
	}

	return *this;
}

template <typename T>
Matrix<T>::~Matrix(void)
{
	if ( m_matrix != NULL ) 
	{
		for ( int i = 0 ; i < m_rows ; i++ )
			delete [] m_matrix[i];

		delete [] m_matrix;
	}
	m_matrix = NULL;
}

template <typename T>
T& Matrix<T>::operator()(int x, int y)
{
	assert ( x >= 0 );
	assert ( y >= 0 );
	assert ( x < m_rows );
	assert ( y < m_columns );
	assert ( m_matrix != NULL );
	return m_matrix[x][y];
}

template <typename T>
T Matrix<T>::trace(void)
{
	assert( m_matrix != NULL );
	T value = 0;

	int x = std::min<int>(m_rows, m_columns);
	for ( int i = 0 ; i < x ; i++ )
		value += m_matrix[i][i];

	return value;
}

template <typename T>
Matrix<T>& Matrix<T>::transpose(void)
{
	assert( m_rows > 0 );
	assert( m_columns > 0 );

	int new_rows = m_columns;
	int new_columns = m_rows;

	if ( m_rows != m_columns ) 
	{
		int m = std::max<int>(m_rows, m_columns);
		resize(m,m);
	}

	for ( int i = 0 ; i < m_rows ; i++ ) 
	{
		for ( int j = i+1 ; j < m_columns ; j++ ) 
		{
			T tmp = m_matrix[i][j];
			m_matrix[i][j] = m_matrix[j][i];
			m_matrix[j][i] = tmp;
		}
	}

	if ( new_columns != new_rows ) 
	{
		resize(new_rows, new_columns);
	}

	return *this;
}

template <typename T>
Matrix<T> Matrix<T>::product(Matrix<T>& other)
{
	assert( m_matrix != NULL );
	assert( other.m_matrix != NULL );
	assert ( m_columns == other.m_rows );

	Matrix<T> out(m_rows, other.m_columns);

	for ( int i = 0 ; i < out.m_rows ; i++ ) 
	{
		for ( int j = 0 ; j < out.m_columns ; j++ ) 
		{
			for ( int x = 0 ; x < m_columns ; x++ ) 
			{
				out(i,j) += m_matrix[i][x] * other.m_matrix[x][j];
			}
		}
	}

	return out;
}