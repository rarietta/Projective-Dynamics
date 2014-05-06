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

#ifndef _SIMULATION_H_
#define _SIMULATION_H_

#include <vector>

#include "global_headers.h"
#include "anttweakbar_wrapper.h"
#include "mesh.h"
#include "constraint.h"
#include "scene.h"

class Mesh;
class AntTweakBarWrapper;

typedef enum
{
	PREFACTOR_L,
	PREFACTOR_M_PLUS_H2L,
	PREFACTOR_M_PLUS_H2QTQ,
	PREFACTOR_M_PLUS_H2QTQ_1D,
	PREFACTOR_TOTAL_NUM

} PrefactorType;

typedef enum
{
	INTEGRATION_PBD,
	INTEGRATION_LOCAL_GLOBAL,
	INTEGRATION_EXPLICIT_EULER,
	INTEGRATION_EXPLICIT_SYMPLECTIC,
	INTEGRATION_IMPLICIT_EULER_BARAFF_WITKIN,
	INTEGRATION_GRADIENT_DESCENT,
	INTEGRATION_NEWTON_DESCENT,
	INTEGRATION_NEWTON_DESCENT_PCG,
	INTEGRATION_TOTAL_NUM

} IntegrationMethod;

class Simulation
{
	friend class AntTweakBarWrapper;

public:
	Simulation();
	virtual ~Simulation();

	void Reset();
	void UpdateAnimation(const int fn);
	void Update();
	void DrawConstraints(const VBO& vbos);

	// projection based dynamics functions
	void CreateLHSMatrix(void);
	void CreateRHSMatrix(void);
	SparseMatrix CreateSMatrix(Constraint* c);

	// select/unselect/move/save/load attachment constratins
	ScalarType TryToSelectAttachmentConstraint(const EigenVector3& p0, const EigenVector3& dir); // return ray_projection_plane_distance if hit; return -1 otherwise.
	bool TryToToggleAttachmentConstraint(const EigenVector3& p0, const EigenVector3& dir); // true if hit some vertex/constraint
	void SelectAtttachmentConstraint(AttachmentConstraint* ac);
	void UnselectAttachmentConstraint();
	void AddAttachmentConstraint(unsigned int vertex_index); // add one attachment constraint at vertex_index
	void MoveSelectedAttachmentConstraintTo(const EigenVector3& target); // move selected attachement constraint to target
	void SaveAttachmentConstraint(const char* filename);
	void LoadAttachmentConstraint(const char* filename);

	// inline functions
	inline IntegrationMethod GetIntegrationMethod() {return m_integration_method; }
	inline void SetReprefactorFlag() 
	{
		//TODO: set prefactorization flag to true if you have prefactored anything
	}
	inline void SetMesh(Mesh* mesh) {m_mesh = mesh;}
	inline void SetScene(Scene* scene) {m_scene = scene;}
	inline void SetStepMode(bool step_mode) {m_step_mode = step_mode;}
	
protected:

	// simulation constants
	ScalarType m_h; // time_step
	bool m_quasi_statics; 
	bool m_step_mode;

	// simulation constants
	ScalarType m_gravity_constant;
	ScalarType m_stiffness_attachment;
	ScalarType m_stiffness_stretch;
	ScalarType m_stiffness_bending;
	ScalarType m_damping_coefficient;
	Eigen::LLT<Matrix> m_llt;
	std::vector<bool> isColliding;

	// integration method
	IntegrationMethod m_integration_method;

	// key simulation components: mesh and scene
	Mesh *m_mesh;
	Scene *m_scene;
	// key simulation components: constraints
	std::vector<Constraint*> m_constraints;
	AttachmentConstraint* m_selected_attachment_constraint;

	// inertia term
	VectorX m_inertia_y;

	// external force (gravity, wind, etc...)
	VectorX m_external_force;

	// for optimization method, number of iterations
	unsigned int m_iterations_per_frame;

	// line search for gradient descent and newton's method
	bool m_enable_line_search;
	ScalarType m_ls_alpha;
	ScalarType m_ls_beta;
	ScalarType m_ls_step_size;

	// local global method
	bool m_enable_openmp;

	// animation for demo
	bool m_animation_enable_swinging;
	int m_animation_swing_num;
	int m_animation_swing_half_period;
	ScalarType m_animation_swing_amp;
	ScalarType m_animation_swing_dir[3];	

	SparseMatrix m_A_attachment;
	SparseMatrix m_A_spring;
	SparseMatrix m_A_tet;
	SparseMatrix m_B_attachment;
	SparseMatrix m_B_spring;
	SparseMatrix m_B_tet;

	VectorX p_attach;
	VectorX p_spring;

private:

	// main update sub-routines
	void clearConstraints(); // cleanup all constraints
	void setupConstraints(); // initialize constraints
	void dampVelocity(); // damp velocity at the end of each iteration.
	void calculateInertiaY(); // calculate the inertia term: y = current_pos + current_vel*h
	void calculateExternalForce(); // wind force is propotional to the area of triangles projected on the tangential plane
	VectorX collisionDetection(const VectorX x); // detect collision and return a vector of penetration

	// there should also be such functions that...
	// handles traditional time integration methods such as EE, Baraff-Witkin, etc..
	// handles optimization implicit method
	// evaluates system energy, system gradient, system hessian, etc...
	
	// Utility
	// your own utility functions to make things easier.
};

#endif