#pragma once
#include "Matrix.h"
#include "Parameter.h"
#include <list>
#include <utility>
#define INFINITY 1000000000


class HungarianAlg
{
public:
	void solve(Matrix<double> &m);
private:
	static const int NORMAL = 0;
	static const int STAR = 1;
	static const int PRIME = 2; 
	bool find_uncovered_in_matrix(double,int&,int&);
	bool pair_in_list(const std::pair<int,int> &, const std::list<std::pair<int,int> > &);

	int step1(void);
	int step2(void);
	int step3(void);
	int step4(void);
	int step5(void);
	int step6(void);
	Matrix<int> mask_matrix;
	Matrix<double> matrix;
	bool *row_mask;
	bool *col_mask;
	int saverow, savecol;
};

