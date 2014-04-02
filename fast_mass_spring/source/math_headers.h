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

#ifndef _MATH_HEADERS_H_
#define _MATH_HEADERS_H_

// eigen
#include "Core"
#include "Dense"
#include "Sparse"

// glm
#include "glm.hpp"
#include "gtc\matrix_transform.hpp"

// global header
#include "global_headers.h"

// eigen vectors and matrices
typedef int IndexType;
typedef Eigen::Matrix<ScalarType, 3, 3, 0, 3 ,3> EigenMatrix3;
typedef Eigen::Matrix<ScalarType, 3, 1, 0, 3 ,1> EigenVector3;
typedef Eigen::Matrix<ScalarType, 2, 2, 0, 2 ,2> EigenMatrix2;
typedef Eigen::Matrix<ScalarType, 2, 1, 0, 2 ,1> EigenVector2;
typedef Eigen::Matrix<ScalarType, -1, 3, 0, -1 ,3> EigenMatrix3x;
typedef Eigen::Matrix<ScalarType, Eigen::Dynamic, 1> VectorX;
typedef Eigen::Matrix<ScalarType, Eigen::Dynamic, Eigen::Dynamic> Matrix;
typedef Eigen::SparseMatrix<ScalarType> SparseMatrix;
typedef Eigen::Triplet<ScalarType,IndexType> SparseMatrixTriplet;

// eigen vector accessor
#define block_vector(a) block<3,1>(3*(a), 0)

// eigen 2 glm, glm 2 eigen
glm::vec3 Eigen2GLM(const EigenVector3& eigen_vector);
EigenVector3 GLM2Eigen(const glm::vec3& glm_vector);

#endif