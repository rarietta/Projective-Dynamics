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

#include <exception>

#include "constraint.h"
#include "SVD"

//----------Utility Functions---------//
void ThreeVector3ToMatrix3(EigenMatrix3& m, EigenVector3& v1, EigenVector3& v2, EigenVector3& v3)
{
	m.block<3, 1>(0, 0) = v1;
	m.block<3, 1>(0, 1) = v2;
	m.block<3, 1>(0, 2) = v3;
}

//----------Constraint Class----------//
Constraint::Constraint(ScalarType *stiffness) : 
	m_stiffness(stiffness)
{
}

Constraint::Constraint(const Constraint& other) : 
	m_stiffness(other.m_stiffness)
{
}

Constraint::~Constraint()
{
}

//----------AttachmentConstraint Class----------//
AttachmentConstraint::AttachmentConstraint(ScalarType *stiffness) : 
	Constraint(stiffness)
{
	m_selected = false;
	constraintType = ATTACHMENT;
}

AttachmentConstraint::AttachmentConstraint(ScalarType *stiffness, unsigned int p0, const EigenVector3& fixedpoint) : 
	Constraint(stiffness),
	m_p0(p0),
	m_fixd_point(fixedpoint)
{
	m_selected = false;
	constraintType = ATTACHMENT;
}

AttachmentConstraint::AttachmentConstraint(const AttachmentConstraint& other) : 
	Constraint(other),
	m_p0(other.m_p0),
	m_fixd_point(other.m_fixd_point),
	m_selected(other.m_selected)
{
	constraintType = ATTACHMENT;
}

AttachmentConstraint::~AttachmentConstraint()
{

}

// 0.5*k*(current_length)^2
ScalarType AttachmentConstraint::EvaluatePotentialEnergy(const VectorX& x)
{
	// TODO
	ScalarType k = *m_stiffness;
	EigenVector3 current_position = x.block_vector(m_p0);
	ScalarType current_length = (current_position - m_fixd_point).norm();
	return 0.5 * k * current_length * current_length * 10;
}

// attachment spring gradient: k*(current_length)*current_direction
void AttachmentConstraint::EvaluateGradient(const VectorX& x, VectorX& gradient)
{
	// TODO
}

void AttachmentConstraint::EvaluateHessian(const VectorX& x, std::vector<SparseMatrixTriplet>& hessian_triplets)
{
	// TODO
}

void AttachmentConstraint::EvaluateWeightedLaplacian(std::vector<SparseMatrixTriplet>& laplacian_triplets)
{
	// TODO
}

void AttachmentConstraint::Draw(const VBO& vbos)
{
	m_attachment_constraint_body.move_to(Eigen2GLM(m_fixd_point));
	if (m_selected)
		m_attachment_constraint_body.change_color(glm::vec3(0.8, 0.8, 0.2));
	else
		m_attachment_constraint_body.change_color(glm::vec3(0.8, 0.2, 0.2));
		
	m_attachment_constraint_body.Draw(vbos);
}

void AttachmentConstraint::WriteToFileOBJ(std::ofstream& outfile, int& existing_vertices)
{
	// assume outfile is correctly open.

	std::vector<glm::vec3>& vertices = m_attachment_constraint_body.GetPositions();
	std::vector<unsigned short>& triangles = m_attachment_constraint_body.GetTriangulation();
	glm::vec3 move_to = Eigen2GLM(m_fixd_point);
	glm::vec3 v;

	for (unsigned int i = 0; i < vertices.size(); i++)
	{
		v = vertices[i] + move_to;
		// save positions
		outfile << "v " << v[0] << " " << v[1] << " " << v[2] << std::endl;
	}
	outfile << std::endl;
	for (unsigned int f = 0; f < triangles.size(); f += 3)
	{
		outfile << "f " << triangles[f+0]+1+existing_vertices << " " << triangles[f+1]+1+existing_vertices << " " << triangles[f+2]+1+existing_vertices << std::endl;
	}
	outfile << std::endl;

	existing_vertices += vertices.size();
}

void AttachmentConstraint::WriteToFileOBJHead(std::ofstream& outfile)
{
	EigenVector3& v = m_fixd_point;
	outfile << "// " << m_p0 << " " << v[0] << " " << v[1] << " " << v[2] << std::endl;
}

//----------SpringConstraint Class----------//
SpringConstraint::SpringConstraint(ScalarType *stiffness) : 
	Constraint(stiffness)
{
	constraintType = SPRING;
}

SpringConstraint::SpringConstraint(ScalarType *stiffness, unsigned int p1, unsigned int p2, ScalarType length) : 
	Constraint(stiffness),
	m_p1(p1),
	m_p2(p2),
	m_rest_length(length)
{
	constraintType = SPRING;
}

SpringConstraint::SpringConstraint(const SpringConstraint& other) : 
	Constraint(other),
	m_p1(other.m_p1),
	m_p2(other.m_p2),
	m_rest_length(other.m_rest_length)
{
	constraintType = SPRING;
}

SpringConstraint::~SpringConstraint()
{
}

// 0.5*k*(current_length - rest_length)^2
ScalarType SpringConstraint::EvaluatePotentialEnergy(const VectorX& x)
{
	// TODO
	ScalarType k = *m_stiffness;
	EigenVector3 current_position_p1 = x.block_vector(m_p1);
	EigenVector3 current_position_p2 = x.block_vector(m_p2);
	ScalarType current_length = (current_position_p1 - current_position_p2).norm();
	ScalarType difference = current_length - m_rest_length;
	return 0.5 * k * difference * difference;
}

// sping gradient: k*(current_length-rest_length)*current_direction;
void SpringConstraint::EvaluateGradient(const VectorX& x, VectorX& gradient)
{
	// TODO
}

void SpringConstraint::EvaluateHessian(const VectorX& x, std::vector<SparseMatrixTriplet>& laplacian_triplets)
{
	// TODO
}

void SpringConstraint::EvaluateWeightedLaplacian(std::vector<SparseMatrixTriplet>& laplacian_triplets)
{
	// TODO
}

//----------TetConstraint Class----------//
TetConstraint::TetConstraint(ScalarType *stiffness) : 
	Constraint(stiffness)
{
	constraintType = TET;
}

TetConstraint::TetConstraint(ScalarType *stiffness, unsigned int p1, unsigned int p2, unsigned int p3, unsigned int p4, VectorX& x) : 
	Constraint(stiffness)
{
	m_p[0] = p1;
	m_p[1] = p2;
	m_p[2] = p3;
	m_p[3] = p4;

	EigenVector3 v1 = x.block_vector(p1)-x.block_vector(p4);
	EigenVector3 v2 = x.block_vector(p2)-x.block_vector(p4);
	EigenVector3 v3 = x.block_vector(p3)-x.block_vector(p4);

	ThreeVector3ToMatrix3(m_Dr, v1, v2, v3);

	m_W = m_Dr.determinant();

	m_W = 1.0/6.0 * std::abs(m_W);

	m_Dr_inv = m_Dr.inverse();

	Eigen::Matrix<ScalarType, 3, 4> IND;
	IND.block<3, 3>(0, 0) = EigenMatrix3::Identity();
	IND.block<3, 1>(0, 3) = EigenVector3(-1, -1, -1);

	m_Q = m_Dr_inv.transpose() * IND;

	m_Q_kron_I33.resize(9, 12);
	constraintType = TET;
}

TetConstraint::TetConstraint(const TetConstraint& other) : 
	Constraint(other)
{
	m_p[0] = other.m_p[0];
	m_p[1] = other.m_p[1];
	m_p[2] = other.m_p[2];
	m_p[3] = other.m_p[3];

	m_Dr = other.m_Dr;
	m_Dr_inv = other.m_Dr_inv;

	m_W = other.m_W;
	m_Q = other.m_Q;

	m_Q_kron_I33 = other.m_Q_kron_I33;
	constraintType = TET;
}

TetConstraint::~TetConstraint()
{
}

// 0.5*mu*||F-R||^2
ScalarType TetConstraint::EvaluatePotentialEnergy(const VectorX& x)
{
	// TODO
	EigenMatrix3 F;
	getDeformationGradient(F, x);

	EigenMatrix3 U = F;
	F.transpose().applyThisOnTheLeft(U);
	U = U.cwiseSqrt();

	EigenMatrix3 R = U.inverse();
	F.applyThisOnTheLeft(R);

	ScalarType mu = *m_stiffness;
	ScalarType returnValue = 0.5 * mu * (F-R).norm() * (F-R).norm(); 
	return returnValue;
}

void TetConstraint::EvaluateGradient(const VectorX& x, VectorX& gradient)
{
	// TODO
}

void TetConstraint::EvaluateHessian(const VectorX& x, std::vector<SparseMatrixTriplet>& hessian_triplets)
{
	// TODO
}

// strain limiting volume preservation
void TetConstraint::computeVolumePreservingVertexPositions( VectorX& new_vertex_positions, const VectorX& current_vertex_positions )
{
	// compute the deformation gradient, F, for the tetrahedron's current deformation
	EigenMatrix3 F;
	getDeformationGradient( F, current_vertex_positions );

	// run SVD to get U, S, and V from the deformation gradient
	EigenMatrix3 U, V;
	EigenVector3 SIGMA;
	singularValueDecomp( U, SIGMA, V, F );

	// compute a new S, S*, by clamping the existing s values (stresses) on the main diagonal of S
	double min = 0.95;
	double max = 1.05;
	EigenMatrix3 SIGMA_new;
	SIGMA_new << clamp( SIGMA( 0, 0 ), min, max ),	0.0,								0.0,
				 0.0,								clamp( SIGMA( 1, 0 ), min, max ),	0.0,
				 0.0,								0.0,								clamp( SIGMA( 2, 0 ), min, max );

	// compute a new deformation gradient, F*, using the existing U and V rotation matrices along with S*
	EigenMatrix3 F_new = V.transpose();
	SIGMA_new.applyThisOnTheLeft( F_new );
	U.applyThisOnTheLeft( F_new );

	// compute deformed basis matrix from F* and rest state basis matrix
	EigenMatrix3 deformed_basis = m_Dr;
	F_new.applyThisOnTheLeft( deformed_basis );

	// use deformed basis matrix to compute new positions of the tetrahedron's vertices
	EigenVector3 tet_centroid = ( current_vertex_positions.block_vector( 0 ) + current_vertex_positions.block_vector( 1 ) + current_vertex_positions.block_vector( 2 ) + current_vertex_positions.block_vector( 3 ) ) / 4.0;
	new_vertex_positions.block_vector( 3 ) = tet_centroid - ( deformed_basis.col( 0 ) + deformed_basis.col( 1 ) + deformed_basis.col( 2 ) ) / 4.0;
	new_vertex_positions.block_vector( 0 ) = new_vertex_positions.block_vector( 3 ) + deformed_basis.col( 0 );
	new_vertex_positions.block_vector( 1 ) = new_vertex_positions.block_vector( 3 ) + deformed_basis.col( 1 );
	new_vertex_positions.block_vector( 2 ) = new_vertex_positions.block_vector( 3 ) + deformed_basis.col( 2 );
}

double TetConstraint::clamp( double n, double lower, double upper )
{
	return std::max( lower, std::min( n, upper ) );
}

void TetConstraint::calculateDPDF(EigenMatrix3 dPdF[][3], const EigenMatrix3& U, const EigenVector3& SIGMA, const EigenMatrix3& V)
{
	// TODO
}

void TetConstraint::getDeformationGradient(EigenMatrix3& F, const VectorX& x)
{
	// TODO
	EigenVector3 x1, x2, x3, x4;
	//x1 = x.block_vector(m_p[0]);
	//x2 = x.block_vector(m_p[1]);
	//x3 = x.block_vector(m_p[2]);
	//x4 = x.block_vector(m_p[3]);
	x1 = x.block_vector( 0 );
	x2 = x.block_vector( 1 );
	x3 = x.block_vector( 2 );
	x4 = x.block_vector( 3 );

	EigenMatrix3 Ds;
	Ds(0,0) = x1.x() - x4.x();	Ds(0,1) = x2.x() - x4.x();	Ds(0,2) = x3.x() - x4.x();
	Ds(1,0) = x1.y() - x4.y();	Ds(1,1) = x2.y() - x4.y();	Ds(1,2) = x3.y() - x4.y();
	Ds(2,0) = x1.z() - x4.z();	Ds(2,1) = x2.z() - x4.z();	Ds(2,2) = x3.z() - x4.z();

	F = m_Dr_inv;
	Ds.applyThisOnTheLeft(F);
}

void TetConstraint::getStressTensor(EigenMatrix3& P, const EigenMatrix3& F, const EigenMatrix3& R)
{
	// TODO
	ScalarType mu = *m_stiffness;
	P = mu * (F - R);
}

void TetConstraint::singularValueDecomp(EigenMatrix3& U, EigenVector3& SIGMA, EigenMatrix3& V, const EigenMatrix3& A)
{
	Eigen::JacobiSVD<EigenMatrix3> svd;
	svd.compute(A, Eigen::ComputeFullU | Eigen::ComputeFullV);

	U = svd.matrixU();
	V = svd.matrixV();
	SIGMA = svd.singularValues();

	ScalarType detU = U.determinant();
	ScalarType detV = V.determinant();

	// make sure that both U and V are rotation matrices without reflection
	if (detU < 0)
	{
		U.block<3,1>(0,2) *= -1;	
		SIGMA[2] *= -1;
	}
	if (detV < 0)
	{
		V.block<3,1>(0,2) *= -1;
		SIGMA[2] *= -1;
	}
}