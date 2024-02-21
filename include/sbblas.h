#pragma once

#include <sbtypes.h>

#include <map>
#include <set>

#define USE_MKL
//#define USE_CUBLAS

#if defined USE_MKL
# ifdef _WIN32
#  include <mkl.h>
//#  include <mkl_lapacke.h>
//#  include <mkl_cblas.h>
# else
#  include <mkl.h>
//#  include <lapacke.h>
# endif
#endif

#if defined USE_CUBLAS
# include <cuda_runtime.h>
# include <cublas_v2.h>
# include <cusolverDn.h>
#endif

namespace Scenebuilder{;

struct Vector{
	int     n;
	double* vh;
	
	void Delete  ();
	void Allocate(int _n);
	void Resize  (int _n);

	double& operator()(int i)     { assert(0 <= i && i < n); return vh[i]; }
	double  operator()(int i)const{ assert(0 <= i && i < n); return vh[i]; }

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

	double& operator()(int i, int j)     { assert(0 <= i && i < m && 0 <= j && j < n); return vh[l*j+i]; }
	double& operator()(int i, int j)const{ assert(0 <= i && i < m && 0 <= j && j < n); return vh[l*j+i]; }

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
	struct PrintCallback{
		virtual bool Order(int lhs, int rhs) = 0;
		virtual int  Label(int i, int j, bool nonzero) = 0;
	};

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

	void  PrintSparsity(ostream& os, PrintCallback* callback = 0);

	SparseMatrix();
};

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

ostream& operator<<(ostream& os, Vector& v);
ostream& operator<<(ostream& os, Matrix& m);

template<size_t N, class D>
void vec_copy(const PTM::TVectorBase<N, D>& v, Vector& y){
	typedef typename D::element_type T;
	assert(y.n == N);
	const T* v0 = &v[0];
	double*  v1 = y.vh;
	for(int i = 0; i < N; i++)
		*v1++ = (double)*v0++;
}

template<class T>
void vec_copy(const PTM::VVector<T>& v, Vector& y){
	assert(y.n == v.size());
	const T* v0 = &v[0];
	double*  v1 = y.vh;
	for(int i = 0; i < y.n; i++)
		*v1++ = (double)*v0++;
}

template<size_t H, size_t W, class D>
void  mat_copy (const PTM::TMatrixBase<H, W, D>& m1, Matrix&& y){
	typedef typename D::element_type T;
	assert(y.m == H && y.n == W);
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
	assert(y.m == m.height() && y.n == m.width());
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

inline void vec_clear(Vector&& y){
	double* vh = y.vh;
	for(int i = 0; i < y.n; i++)
		*vh++ = 0.0;
}
inline void vec_clear(Vector& y){
	vec_clear((Vector&&)std::move(y));
}

inline void vec_clear(SparseVector& y){
	for(SparseVector::iterator it = y.begin(); it != y.end(); it++)
		vec_clear(it->second);
}

inline double vec_norm(const Vector& v){
	double vn = 0.0;
	for(int j = 0; j < v.n; j++)
		vn += (v(j)*v(j));

	return sqrt(vn);
}

inline void mat_clear(Matrix&& y){
	double* vh0 = y.vh;
	for(int j = 0; j < y.n; j++, vh0 += y.l){
		double* vh = vh0;
		for(int i = 0; i < y.m; i++, vh++){
			*vh = 0.0;
		}
	}
}
inline void mat_clear(Matrix& y){
	mat_clear((Matrix&&)std::move(y));
}

inline void mat_clear(SparseMatrix& y){
	for(int i = 0; i < y.m; i++){
		for(SparseMatrix::Row::iterator it = y.rows[i].begin(); it != y.rows[i].end(); it++)
			mat_clear(it->second);
	}
}

inline void vec_copy(const Vector& v1, Vector&& y){
	assert(y.n == v1.n);
	double* vh0 = v1.vh;
	double* vh1 = y .vh;
	for(int i = 0; i < v1.n; i++)
		*vh1++ = *vh0++;
}
inline void vec_copy(const Vector& v1, Vector& y){
	vec_copy(v1, (Vector&&)std::move(y));
}

inline void vec_copy (const SparseVector& v1, SparseVector& y){
	for(SparseVector::const_iterator it = v1.begin(); it != v1.end(); it++){
		int           j  = it->first;
		const Vector& vj = it->second;
		Vector yj = y.SubVector(j);
		vec_copy(vj, yj);
	}
}

template<size_t N, class D>
inline void subvec_copy(const PTM::TVectorBase<N, D>& v, Vector& y, real_t k, int i0){
	int n = (int)v.size();
	assert(0 <= i0 && i0+n <= y.n);
	const double* vh0 = &v[0];
	double* vh1 = &y(i0);
	for(int i = 0; i < n; i++)
		*vh1++ = *vh0++;
}

template<class T>
inline void subvec_copy(const PTM::VVector<T>& v, Vector& y, real_t k, int i0){
	int n = (int)v.size();
	assert(0 <= i0 && i0+n <= y.n);
	const double* vh0 = &v[0];
	double* vh1 = &y(i0);
	for(int i = 0; i < n; i++)
		*vh1++ = *vh0++;
}


inline void mat_copy(const Matrix& m1, Matrix&& y){
	assert(y.m == m1.m && y.n == m1.n);
	double* col0 = m1.vh;
	double* col1 = y .vh;
	for(int j = 0; j < m1.n; j++, col0 += m1.l, col1 += y.l){
		double* v0 = col0;
		double* v1 = col1;
		for(int i = 0; i < m1.m; i++){
			*v1++ = *v0++;
		}
	}
}
inline void mat_copy(const Matrix& m1, Matrix& y){
	mat_copy(m1, (Matrix&&)std::move(y));
}

inline void mat_copy (const SparseMatrix& m1, SparseMatrix& y){
	for(int i = 0; i < m1.m; i++){
		for(SparseMatrix::Row::const_iterator it = m1.rows[i].begin(); it != m1.rows[i].end(); it++){
			int           j   = it->first ;
			const Matrix& mij = it->second;
			Matrix yij = y.SubMatrix(i, j);
			mat_copy(mij, yij);
		}
	}
}

template<size_t H, size_t W, class D>
inline void submat_copy(const PTM::TMatrixBase<H, W, D>& m, Matrix& y, real_t k, int r0, int c0){
	int nrow = (int)m.height();
	int ncol = (int)m.width ();
	assert(0 <= r0 && r0+nrow <= y.m && 0 <= c0 && c0+ncol <= y.n);
	const double* col0 = &m[0][0];
	double* col1 = &y(r0, c0);
	for(int j = 0; j < ncol; j++, col0 += nrow, col1 += y.l){
		const double* v0 = col0;
		double* v1 = col1;
		for(int i = 0; i < nrow; i++){
			*v1++ = k*(*v0++);
		}
	}
}

template<class T>
inline void submat_copy(const PTM::VMatrixCol<T>& m, Matrix& y, real_t k, int r0, int c0){
	int nrow = (int)m.height();
	int ncol = (int)m.width ();
	assert(0 <= r0 && r0+nrow <= y.m && 0 <= c0 && c0+ncol <= y.n);
	const double* col0 = &m[0][0];
	double* col1 = &y(r0, c0);
	for(int j = 0; j < ncol; j++, col0 += nrow, col1 += y.l){
		const double* v0 = col0;
		double* v1 = col1;
		for(int i = 0; i < nrow; i++){
			*v1++ = k*(*v0++);
		}
	}
}


inline void vec_add(const Vector& v1, Vector&& y){
	assert(y.n == v1.n);
	double* vh0 = v1.vh;
	double* vh1 = y .vh;
	for(int i = 0; i < v1.n; i++)
		*vh1++ += *vh0++;
}
inline void vec_add(const Vector& v1, Vector& y){
	vec_add(v1, (Vector&&)std::move(y));
}

inline void vec_add(const SparseVector& v1, SparseVector& y){
	for(SparseVector::const_iterator it = v1.begin(); it != v1.end(); it++){
		int           j  = it->first;
		const Vector& vj = it->second;
		Vector yj = y.SubVector(j);
		vec_add(vj, yj);
	}
}

// y += m1
inline void mat_add(const Matrix& m1, Matrix&& y){
	assert(y.m == m1.m && y.n == m1.n);
	double* col0 = m1.vh;
	double* col1 = y .vh;
	for(int j = 0; j < m1.n; j++, col0 += m1.l, col1 += y.l){
		double* v0 = col0;
		double* v1 = col1;
		for(int i = 0; i < m1.m; i++){
			*v1++ += *v0++;
		}
	}
}
// y += k*m1
inline void mat_add(const Matrix& m1, Matrix&& y, double k){
	assert(y.m == m1.m && y.n == m1.n);
	double* col0 = m1.vh;
	double* col1 = y .vh;
	for(int j = 0; j < m1.n; j++, col0 += m1.l, col1 += y.l){
		double* v0 = col0;
		double* v1 = col1;
		for(int i = 0; i < m1.m; i++){
			*v1++ += k*(*v0++);
		}
	}
}
// y += m1^T
inline void mattr_add(const Matrix& m1, Matrix&& y){
	assert(y.m == m1.n && y.n == m1.m);
	double* col0 = m1.vh;
	double* row1 = y .vh;
	for(int j = 0; j < m1.n; j++, col0 += m1.l, row1++){
		double* v0 = col0;
		double* v1 = row1;
		for(int i = 0; i < m1.m; i++){
			*v1 += *v0++;
			v1 += y.l;
		}
	}
}
inline void mat_add(const Matrix& m1, Matrix& y){
	mat_add(m1, (Matrix&&)std::move(y));
}
inline void mat_add(const Matrix& m1, Matrix& y, double k){
	mat_add(m1, (Matrix&&)std::move(y), k);
}
inline void mattr_add(const Matrix& m1, Matrix& y){
	mattr_add(m1, (Matrix&&)std::move(y));
}

inline void mat_add(const SparseMatrix& m1, SparseMatrix& y){
	for(int i = 0; i < m1.m; i++){
		for(SparseMatrix::Row::const_iterator it = m1.rows[i].begin(); it != m1.rows[i].end(); it++){
			int           j   = it->first ;
			const Matrix& mij = it->second;
			Matrix yij = y.SubMatrix(i, j);
			mat_add(mij, yij);
		}
	}
}

inline double mat_abs(const Matrix& m){
	// assumes m.m == m.l
	return cblas_dasum(m.m*m.n, m.vh, 1);
}

inline double vec_dot(const Vector& v1, const Vector& v2){
	assert(v1.n == v2.n);
	return cblas_ddot(v1.n, v1.vh, 1, v2.vh, 1);
}

inline double quadform(const Matrix& m, const Vector& v1, const Vector& v2){
	assert(v1.n == m.m && v2.n == m.n);
	double y = 0.0;
	for(int i = 0; i < v1.n; i++)for(int j = 0; j < v2.n; j++)
		y += m(i,j)*v1(i)*v2(j);

	return y;
}

inline void mat_vec_mul(const Matrix& m1, const Vector& v, Vector&& y, double alpha, double beta){
	assert(y.n == m1.m && v.n == m1.n);
	Vector tmp;
	if(v.vh == y.vh){
		if(tmp.n != y.n)
			tmp.Allocate(y.n);
		vec_copy(y, tmp);
		mat_vec_mul(m1, v, std::move(tmp), alpha, beta);
		vec_copy(tmp, y);
	}
	else{
		cblas_dgemv(CblasColMajor, CblasNoTrans, m1.m, m1.n, alpha, m1.vh, m1.l, v.vh, 1, beta, y.vh, 1);
	}
}
inline void mat_vec_mul(const Matrix& m1, const Vector& v, Vector& y, double alpha, double beta){
	mat_vec_mul(m1, v, (Vector&&)std::move(y), alpha, beta);
}

inline void mat_vec_mul(const SparseMatrix& m1, const SparseVector& v, Vector& y, double alpha, double beta){
	for(SparseMatrix::Row::const_iterator it = m1.rows[0].begin(); it != m1.rows[0].end(); it++){
		int           j   = it->first ;
		const Matrix& m1j = it->second;
		const Vector* vj  = v.Item(j);
		if(vj)
			mat_vec_mul(m1j, *vj, y, alpha, (it == m1.rows[0].begin() ? beta : 1.0));
	}
}

inline void mattr_vec_mul(const Matrix& m1, const Vector& v, Vector&& y, double alpha, double beta){
	assert(y.n == m1.n && v.n == m1.m);
	Vector tmp;
	if(v.vh == y.vh){
		if(tmp.n != y.n)
			tmp.Allocate(y.n);
		vec_copy(y, tmp);
		mattr_vec_mul(m1, v, std::move(tmp), alpha, beta);
		vec_copy(tmp, y);
	}
	else{
		cblas_dgemv(CblasColMajor, CblasTrans, m1.m, m1.n, alpha, m1.vh, m1.l, v.vh, 1, beta, y.vh, 1);
	}
}
inline void mattr_vec_mul(const Matrix& m1, const Vector& v, Vector& y, double alpha, double beta){
	mattr_vec_mul(m1, v, (Vector&&)std::move(y), alpha, beta);
}

inline void mattr_vec_mul(const SparseMatrix& m1, const Vector& v, SparseVector& y, double alpha, double beta){
	for(SparseMatrix::Row::const_iterator it = m1.rows[0].begin(); it != m1.rows[0].end(); it++){
		int           j  = it->first ;
		const Matrix& mj = it->second;
		Vector yj = y.SubVector(j);
		mattr_vec_mul(mj, v, yj, alpha, beta);
	}
}

inline void symmat_vec_mul(const Matrix& m1, const Vector& v, Vector& y, double alpha, double beta){
	assert(y.n == m1.m && v.n == m1.n);
	cblas_dsymv(CblasColMajor, CblasUpper, m1.m, alpha, m1.vh, m1.l, v.vh, 1, beta, y.vh, 1);
}

inline void mat_mat_mul(const Matrix& m1, const Matrix& m2, Matrix&& y, double alpha, double beta){
	assert(y.m == m1.m && y.n == m2.n && m1.n == m2.m);
	Matrix tmp;
	if(m1.vh == y.vh || m2.vh == y.vh){
		if(tmp.m != y.m || tmp.n != y.n)
			tmp.Allocate(y.m, y.n);
		mat_copy(y, tmp);
		mat_mat_mul(m1, m2, std::move(tmp), alpha, beta);
		mat_copy(tmp, y);
	}
	else{
		cblas_dgemm(CblasColMajor, CblasNoTrans, CblasNoTrans, m1.m, m2.n, m2.m, alpha, m1.vh, m1.l, m2.vh, m2.l, beta, y.vh, y.l);
	}
}
inline void mat_mat_mul(const Matrix& m1, const Matrix& m2, Matrix& y, double alpha, double beta){
	mat_mat_mul(m1, m2, (Matrix&&)std::move(y), alpha, beta);
}

//void mat_mat_mul(const SparseMatrix& m1, const Matrix& m2, SparseMatrix& y, double alpha, double beta){
//	for(SparseMatrix::Row::const_iterator it = m1.begin(); it != m1.end(); it++){
//		int           i  = it->first.first;
//		const Matrix& mi = it->second;
//		mat_mat_mul(mi, m2, y.SubMatrix(i, 0), alpha, beta);
//	}
//}

inline void mattr_mat_mul(const Matrix& m1, const Matrix& m2, Matrix&& y, double alpha, double beta){
	assert(y.m == m1.n && y.n == m2.n && m1.m == m2.m);
	Matrix tmp;
	if(m1.vh == y.vh || m2.vh == y.vh){
		if(tmp.m != y.m || tmp.n != y.n)
			tmp.Allocate(y.m, y.n);
		mat_copy(y, tmp);
		mattr_mat_mul(m1, m2, std::move(tmp), alpha, beta);
		mat_copy(tmp, y);
	}
	else{
		cblas_dgemm(CblasColMajor, CblasTrans, CblasNoTrans, m1.n, m2.n, m1.m, alpha, m1.vh, m1.l, m2.vh, m2.l, beta, y.vh, y.l);
	}
}
inline void mattr_mat_mul(const Matrix& m1, const Matrix& m2, Matrix& y, double alpha, double beta){
	mattr_mat_mul(m1, m2, (Matrix&&)std::move(y), alpha, beta);
}

inline void mattr_mat_mul (const Matrix& m1, const SparseMatrix& m2, SparseMatrix& y, double alpha, double beta){
	for(SparseMatrix::Row::const_iterator it = m2.rows[0].begin(); it != m2.rows[0].end(); it++){
		int           j  = it->first ;
		const Matrix& mj = it->second;
		mattr_mat_mul(m1, mj, y.SubMatrix(0, j), alpha, beta);
	}
}

inline void mattr_mat_mul (const SparseMatrix& m1, const SparseMatrix& m2, SparseMatrix& y, double alpha, double beta){
	for(SparseMatrix::Row::const_iterator it1 = m1.rows[0].begin(); it1 != m1.rows[0].end(); it1++)
	for(SparseMatrix::Row::const_iterator it2 = m2.rows[0].begin(); it2 != m2.rows[0].end(); it2++){
		int           i  = it1->first ;
		int           j  = it2->first ;
		const Matrix& mi = it1->second;
		const Matrix& mj = it2->second;
		mattr_mat_mul(mi, mj, y.SubMatrix(i, j), alpha, beta);
	}
}

inline void mat_mattr_mul(const Matrix& m1, const Matrix& m2, Matrix& y, double alpha, double beta){
	assert(y.m == m1.m && y.n == m2.m && m1.n == m2.n);
	cblas_dgemm(CblasColMajor, CblasNoTrans, CblasTrans, m1.m, m2.m, m1.n, alpha, m1.vh, m1.l, m2.vh, m2.l, beta, y.vh, y.l);
}

inline void symmat_mat_mul(const Matrix& m1, const Matrix& m2, Matrix&& y, double alpha, double beta){
	assert(y.m == m1.m && y.n == m2.n && m1.n == m2.m);
	cblas_dsymm(CblasColMajor, CblasLeft, CblasUpper, m1.m, m2.n, alpha, m1.vh, m1.l, m2.vh, m2.l , beta, y.vh, y.l);
}
inline void symmat_mat_mul(const Matrix& m1, const Matrix& m2, Matrix& y, double alpha, double beta){
	symmat_mat_mul(m1, m2, (Matrix&&)std::move(y), alpha, beta);
}

inline void symmat_mat_mul(const Matrix& m1, const SparseMatrix& m2, SparseMatrix& y, double alpha, double beta){
	for(SparseMatrix::Row::const_iterator it = m2.rows[0].begin(); it != m2.rows[0].end(); it++){
		int           j  = it->first ;
		const Matrix& mj = it->second;
		symmat_mat_mul(m1, mj, y.SubMatrix(0, j), alpha, beta);
	}
}

inline void  mat_eye(Matrix& m){
	assert(m.m == m.n);
	for(int j = 0; j < m.n; j++)for(int i = 0; i < m.m; i++)
		m(i,j) = (i == j ? 1.0f : 0.0f);
}

inline void  mat_diag(const Vector& v, Matrix& m){
	m.Allocate(v.n, v.n);
	for(int j = 0; j < m.n; j++)for(int i = 0; i < m.m; i++)
		m(i,j) = (i == j ? v.vh[i] : 0.0f);
}

inline void  mat_eig(const Matrix& m, Vector& wr, Vector& wi, Matrix& vl, Matrix& vr){
	assert(m.m == m.n);
	wr.Allocate(m.m);
	wi.Allocate(m.m);
	vl.Allocate(m.m, m.m);
	vr.Allocate(m.m, m.m);
	Matrix tmp;
	tmp.Allocate(m.m, m.m);
	mat_copy(m, tmp);
	LAPACKE_dgeev(LAPACK_COL_MAJOR, 'V', 'V', m.m, &tmp.vh[0], tmp.l, &wr.vh[0], &wi.vh[0], &vl.vh[0], vl.l, &vr.vh[0], vr.l);
}

// inverse of general matrix
inline void mat_inv_gen(const Matrix& m, Matrix& y){
	assert(m.m == m.n && y.m == m.m && y.n == m.n);
	mat_copy(m, y);
	vector<int> pivot; pivot.resize(m.m);
	LAPACKE_dgetrf(LAPACK_COL_MAJOR, m.m, m.m, &y.vh[0], y.l, &pivot[0]);
	LAPACKE_dgetri(LAPACK_COL_MAJOR, m.m,      &y.vh[0], y.l, &pivot[0]);
}

// inverse of symmetric positive definite matrix
inline void mat_inv_pd(const Matrix& m, Matrix& y){
	assert(m.m == m.n && y.m == m.m && y.n == m.n);
	mat_copy(m, y);
	LAPACKE_dpotrf(LAPACK_COL_MAJOR, 'U', m.m, y.vh, y.l);
	LAPACKE_dpotri(LAPACK_COL_MAJOR, 'U', m.m, y.vh, y.l);
}

/*
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
*/

}
