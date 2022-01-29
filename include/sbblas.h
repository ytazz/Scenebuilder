#pragma once

#include <sbtypes.h>

#include <map>
#include <set>

namespace Scenebuilder{;

struct Vector{
	int     n;
	double* vh;
	
	void Delete  ();
	void Allocate(int _n);
	void Resize  (int _n);

	double& operator()(int i){ return vh[i]; }

	Vector SubVector(int ofst, int sz);

	 Vector();
	 Vector(int _n);
	~Vector();
};

struct Matrix{
	int     m, n, l;  //< rows, cols, leading-dim
	double* vh;
	
	void Delete  ();
	void Allocate(int _m, int _n);
	void Resize  (int _m, int _n);

	double& operator()(int i, int j){ return vh[l*j+i]; }

	Matrix SubMatrix(int row, int col, int _m, int _n);

	 Matrix();
	 Matrix(int _m, int _n);
	~Matrix();
};

struct SparseVector : std::map<int, Vector>{
	int n;
	int dim;
	//std::vector< std::map<int, Vector>::iterator >  index;

	void          Resize   (int _n, int _dim);
	const Vector* Item     (int i) const;
	Vector        SubVector(int i);
	void          Clear    (int i);
	void          Clear    ();
};

struct SparseMatrix{
	int           m;
	int           n;
	int           dim;

	struct Row : std::map<int, Matrix>{
		//std::vector< std::map<int, Matrix>::iterator >  index;
	};
	vector<Row>     rows;   //< number of non-zeros of each row
	
	void           Resize   (int _m, int _n, int _dim);
	const Matrix*  Item     (int i, int j) const;
	Matrix         SubMatrix(int i, int j);
	Matrix         SubMatrix(int i, int j) const;
	SparseMatrix   RowMatrix(int i) const;
	SparseMatrix   ColMatrix(int j) const;
	void           Clear    ();
	void           RowClear (int i);
	void           ColClear (int j);

	void  PrintSparsity(ostream& os);

	SparseMatrix();
};

ostream& operator<<(ostream& os, Vector& v);
ostream& operator<<(ostream& os, Matrix& m);

void  mat_eye    (Matrix& m);
void  mat_diag   (const Vector& v , Matrix& y);
void  mat_eig    (const Matrix& m , Vector& wr, Vector& wi, Matrix& vl, Matrix& vr);
void  mat_inv_gen(const Matrix& m , Matrix& y);
void  mat_inv_pd (const Matrix& m , Matrix& y);

void  vec_clear(Vector      &  y);
void  vec_clear(Vector      && y);
void  vec_clear(SparseVector&  y);
void  mat_clear(Matrix      &  y);
void  mat_clear(Matrix      && y);
void  mat_clear(SparseMatrix&  y);
void  vec_copy (const Vector      & v1, Vector      &  y);
void  vec_copy (const Vector      & v1, Vector      && y);
void  vec_copy (const SparseVector& v1, SparseVector&  y);
void  mat_copy (const Matrix      & m1, Matrix      &  y);
void  mat_copy (const Matrix      & m1, Matrix      && y);
void  mat_copy (const SparseMatrix& m1, SparseMatrix&  y);

template<size_t N, class D>
void vec_copy(const PTM::TVectorBase<N, D>& v, Vector& y){
	typedef typename D::element_type T;
	const T* v0 = &v[0];
	double*  v1 = y.vh;
	for(int i = 0; i < N; i++)
		*v1++ = (double)*v0++;
}

template<class T>
void vec_copy(const PTM::VVector<T>& v, Vector& y){
	const T* v0 = &v[0];
	double*  v1 = y.vh;
	for(int i = 0; i < y.n; i++)
		*v1++ = (double)*v0++;
}

template<size_t H, size_t W, class D>
void  mat_copy (const PTM::TMatrixBase<H, W, D>& m1, Matrix&& y){
	typedef typename D::element_type T;
	const T* col0 = &m1[0][0];
	double*  col1 = y .vh;
	for(int j = 0; j < W; j++, col0 += H, col1 += y.l){
		const real_t* v0 = col0;
		double*       v1 = col1;
		for(int i = 0; i < H; i++){
			*v1++ = (double)*v0++;
		}
	}
}
template<size_t H, size_t W, class D>
void  mat_copy (const PTM::TMatrixBase<H, W, D>& m1, Matrix& y){
	mat_copy(m1, std::move(y));
}

template<class T>
void mat_copy(const PTM::VMatrixCol<T>& m, Matrix& y){
	const T* col0 = &m[0][0];
	double*  col1 = y.vh;
	for(int j = 0; j < y.n; j++, col0 += y.m, col1 += y.l){
		const T* v0 = col0;
		double*  v1 = col1;
		for(int i = 0; i < y.m; i++){
			*v1++ = (double)*v0++;
		}
	}
}

double mat_abs       (const Matrix& m);
void   vec_add       (const Vector&       v1, Vector&       y);
void   vec_add       (const Vector&       v1, Vector&&      y);
void   vec_add       (const SparseVector& v1, SparseVector& y);
void   mat_add       (const Matrix&       m1, Matrix&       y);
void   mat_add       (const Matrix&       m1, Matrix&&      y);
void   mat_add       (const SparseMatrix& m1, SparseMatrix& y);
double vec_dot       (const Vector&       v1, const Vector&       v2);
void   mat_vec_mul   (const Matrix&       m1, const Vector&        v, Vector&       y, double alpha, double beta);
void   mat_vec_mul   (const Matrix&       m1, const Vector&        v, Vector&&      y, double alpha, double beta);
void   mat_vec_mul   (const SparseMatrix& m1, const SparseVector&  v, Vector&       y, double alpha, double beta);
void   symmat_vec_mul(const Matrix&       m1, const Vector&        v, Vector&       y, double alpha, double beta);
void   mattr_vec_mul (const Matrix&       m1, const Vector&        v, Vector&       y, double alpha, double beta);
void   mattr_vec_mul (const Matrix&       m1, const Vector&        v, Vector&&      y, double alpha, double beta);
void   mattr_vec_mul (const SparseMatrix& m1, const Vector&        v, SparseVector& y, double alpha, double beta);
void   mat_mat_mul   (const Matrix&       m1, const Matrix&       m2, Matrix&       y, double alpha, double beta);
void   mat_mat_mul   (const Matrix&       m1, const Matrix&       m2, Matrix&&      y, double alpha, double beta);
//void   mat_mat_mul   (const SparseMatrix& m1, const Matrix&       m2, SparseMatrix& y, double alpha, double beta);
void   mattr_mat_mul (const Matrix&       m1, const Matrix&       m2, Matrix&       y, double alpha, double beta);
void   mattr_mat_mul (const Matrix&       m1, const Matrix&       m2, Matrix&&      y, double alpha, double beta);
void   mattr_mat_mul (const Matrix&       m1, const SparseMatrix& m2, SparseMatrix& y, double alpha, double beta);
void   mattr_mat_mul (const SparseMatrix& m1, const SparseMatrix& m2, SparseMatrix& y, double alpha, double beta);
void   mat_mattr_mul (const Matrix&       m1, const Matrix&       m2, Matrix&       y, double alpha, double beta);
void   symmat_mat_mul(const Matrix&       m1, const Matrix&       m2, Matrix&       y, double alpha, double beta);
void   symmat_mat_mul(const Matrix&       m1, const Matrix&       m2, Matrix&&      y, double alpha, double beta);
void   symmat_mat_mul(const Matrix&       m1, const SparseMatrix& m2, SparseMatrix& y, double alpha, double beta);
void mattr_mat_mul_batch(const Matrix& m1, const Matrix& m2, Matrix& y, double alpha, double beta);

class LinearSolver : public UTRefCount{
public:
	virtual void Init  (SparseMatrix& A) = 0;
	virtual void Finish() = 0;
	virtual void Solve (SparseMatrix& A, SparseVector& b, SparseVector& x) = 0;

};

class LinearSolverCustom : public LinearSolver{
public:
	bool mindeg;

	vector<int> order;
	set<int> queue;
	vector<Matrix>        Aii;
	vector<Matrix>        Aii_inv;
	vector<SparseMatrix>  Arow;
	vector<Vector>        bi;
	vector<Vector>        Aii_inv_bi;
	vector<SparseMatrix>  Aii_inv_Arow;

public:
	virtual void Init  (SparseMatrix& A);
	virtual void Finish();
	virtual void Solve (SparseMatrix& A, SparseVector& b, SparseVector& x);

	LinearSolverCustom();
};

class LinearSolverCholmod : public LinearSolver{
public:
	int nrow;
	int ncol;
	int nblock;
	int nzmax;

	void* work;

public:
	virtual void Init  (SparseMatrix& A);
	virtual void Finish();
	virtual void Solve (SparseMatrix& A, SparseVector& b, SparseVector& x);

	LinearSolverCholmod();
};

}
