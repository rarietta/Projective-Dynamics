// ---------------------------------------------------------------------------------//
// Copyright (c) 2013, Regents of the University of Pennsylvania                    //
// All rights reserved.                                                             //
//                                                                                  //
// Redistribution and use in source and binary forms, with or without               //
// modification, are permitted provided that the following conditions are met:      //
//     * Redistributions of source code must retain the above copyright             //
//       notice, this list of conditions and the following disclaimer.              //
//     * Redistributions in binary form must reproduce the above copyright          //
//       notice, this list of conditions and the following disclaimer in the        //
//       documentation and/or other materials provided with the distribution.       //
//     * Neither the name of the <organization> nor the                             //
//       names of its contributors may be used to endorse or promote products       //
//       derived from this software without specific prior written permission.      //
//                                                                                  //
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND  //
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED    //
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE           //
// DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY               //
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES       //
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;     //
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND      //
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT       //
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS    //
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.                     //
//                                                                                  //
// Contact Tiantian Liu (ltt1598@gmail.com) if you have any questions.              //
//----------------------------------------------------------------------------------//

#ifndef _CONSTRAINT_H_
#define _CONSTRAINT_H_

#include <vector>
#include <iostream>
#include <fstream>

#include "global_headers.h"
#include "math_headers.h"
#include "opengl_headers.h"
#include "primitive.h"

class Constraint
{
public:
	Constraint(ScalarType *stiffness);
	Constraint(const Constraint& other);
	virtual ~Constraint();

	virtual ScalarType  EvaluatePotentialEnergy(const VectorX& x) {std::cout << "Warning: reach <Constraint::EvaluatePotentialEnergy> base class virtual function." << std::endl; return 0;}
	virtual void  EvaluateGradient(const VectorX& x, VectorX& gradient) {std::cout << "Warning: reach <Constraint::EvaluateGradient> base class virtual function." << std::endl;}
	virtual void  EvaluateHessian(const VectorX& x, std::vector<SparseMatrixTriplet>& hessian_triplets) {std::cout << "Warning: reach <Constraint::EvaluateHessian> base class virtual function." << std::endl;}
	virtual void  EvaluateWeightedLaplacian(std::vector<SparseMatrixTriplet>& laplacian_triplets) {std::cout << "Warning: reach <Constraint::EvaluateWeightedLaplacian> base class virtual function." << std::endl;}
	
	inline const ScalarType& Stiffness() {return (*m_stiffness);}

protected:
	ScalarType *m_stiffness;

// for visualization and selection
public:
	virtual void Draw(const VBO& vbos) { /*do nothing*/ }
	virtual void WriteToFileOBJ(std::ofstream& outfile, int& existing_vertices) { /*do nothing*/ }
	virtual void WriteToFileOBJHead(std::ofstream& outfile) { /*do nothing*/ }
	//virtual ScalarType RayConstraintIntersection() {return false;}
};

class AttachmentConstraint : public Constraint
{
public:
	AttachmentConstraint(ScalarType *stiffness);
	AttachmentConstraint(ScalarType *stiffness, unsigned int p0, const EigenVector3& fixedpoint);
	AttachmentConstraint(const AttachmentConstraint& other);
	virtual ~AttachmentConstraint();

	virtual ScalarType  EvaluatePotentialEnergy(const VectorX& x);
	virtual void  EvaluateGradient(const VectorX& x, VectorX& gradient);
	virtual void  EvaluateHessian(const VectorX& x, std::vector<SparseMatrixTriplet>& hessian_triplets);
	virtual void  EvaluateWeightedLaplacian(std::vector<SparseMatrixTriplet>& laplacian_triplets);

protected:
	unsigned int m_p0;
	EigenVector3 m_g;
	EigenVector3 m_fixd_point;

// for visualization and selection
public:
	virtual void Draw(const VBO& vbos);
	virtual void WriteToFileOBJ(std::ofstream& outfile, int& existing_vertices);
	virtual void WriteToFileOBJHead(std::ofstream& outfile);
	inline void Select() {m_selected = true;}
	inline void UnSelect() {m_selected = false;}
	inline EigenVector3 GetFixedPoint() {return m_fixd_point;}
	inline void SetFixedPoint(const EigenVector3& target) {m_fixd_point = target;}
	inline unsigned int GetConstrainedVertexIndex() {return m_p0;}

private: 
	bool m_selected;
	Sphere m_attachment_constraint_body;
};

class SpringConstraint : public Constraint
{
public:
	SpringConstraint(ScalarType *stiffness);
	SpringConstraint(ScalarType *stiffness, unsigned int p1, unsigned int p2, ScalarType length);
	SpringConstraint(const SpringConstraint& other);
	virtual ~SpringConstraint();

	virtual ScalarType  EvaluatePotentialEnergy(const VectorX& x);
	virtual void  EvaluateGradient(const VectorX& x, VectorX& gradient);
	virtual void  EvaluateHessian(const VectorX& x, std::vector<SparseMatrixTriplet>& hessian_triplets);
	virtual void  EvaluateWeightedLaplacian(std::vector<SparseMatrixTriplet>& laplacian_triplets);

protected:
	unsigned int m_p1, m_p2;
	EigenVector3 m_g1, m_g2;
	// rest length
	ScalarType m_rest_length;
};

class TetConstraint : public Constraint
{
public:
	TetConstraint(ScalarType *stiffness);
	TetConstraint(ScalarType *stiffness, unsigned int p1, unsigned int p2, unsigned int p3, unsigned int p4, VectorX& x);
	TetConstraint(const TetConstraint& other);
	virtual ~TetConstraint();

	virtual ScalarType  EvaluatePotentialEnergy(const VectorX& x);
	virtual void  EvaluateGradient(const VectorX& x, VectorX& gradient);
	virtual void  EvaluateHessian(const VectorX& x, std::vector<SparseMatrixTriplet>& hessian_triplets);

private:
	void getDeformationGradient(EigenMatrix3& F, const VectorX& x);
	void getStressTensor(EigenMatrix3& P, const EigenMatrix3& F, const EigenMatrix3& R);
	void singularValueDecomp(EigenMatrix3& U, EigenVector3& SIGMA, EigenMatrix3& V, const EigenMatrix3& A);

	void calculateDPDF(EigenMatrix3 dPdF[][3], const EigenMatrix3& U, const EigenVector3& SIGMA, const EigenMatrix3& V);

protected:
	unsigned int m_p[4]; // indices of four vertices
	EigenVector3 m_g[4];
	EigenMatrix3 m_Dr; // [x1-x4|x2-x4|x3-x4]
	EigenMatrix3 m_Dr_inv; // inverse of m_Dr
	Eigen::Matrix<ScalarType, 3, 4> m_Q; // Q = m_Dr^(-T) * IND;
	SparseMatrix m_Q_kron_I33;
	ScalarType m_W; // 1/6 det(Dr);
};

#endif