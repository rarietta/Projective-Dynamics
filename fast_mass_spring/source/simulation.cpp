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

#pragma warning( disable : 4996)
#include <omp.h>
#include <exception>

#include "simulation.h"
#include "timer_wrapper.h"

TimerWrapper g_integration_timer;

Simulation::Simulation()
{

}

Simulation::~Simulation()
{
	clearConstraints();
}

void Simulation::Reset()
{	
	m_inertia_y.resize(m_mesh->m_system_dimension);
	m_external_force.resize(m_mesh->m_system_dimension);
	
	m_mesh->m_expanded_system_dimension = 0;
	m_mesh->m_expanded_system_dimension_1d = 0;

	setupConstraints();
	SetReprefactorFlag();

	m_selected_attachment_constraint = NULL;
	m_step_mode = false;
}

void Simulation::UpdateAnimation(const int fn)
{
	if (m_animation_enable_swinging)
	{
		int swing_num = 0;
		ScalarType swing_step = m_animation_swing_amp / m_animation_swing_half_period;
		EigenVector3 swing_dir(m_animation_swing_dir[0], m_animation_swing_dir[1], m_animation_swing_dir[2]);
		int positive_direction = ((fn/m_animation_swing_half_period)%2)?-1:1;
		for (std::vector<Constraint*>::iterator c = m_constraints.begin(); c != m_constraints.end(); ++c)
		{
			AttachmentConstraint* ac;
			if (ac = dynamic_cast<AttachmentConstraint*>(*c)) // is attachment constraint
			{
				EigenVector3 new_fixed_point = ac->GetFixedPoint() + swing_dir*swing_step*positive_direction;
				ac->SetFixedPoint(new_fixed_point);
				if (++swing_num >= m_animation_swing_num)
				{
					break;
				}
			}
		}
	}
}

void Simulation::Update()
{
	// update inertia term
	calculateInertiaY();

	// update external force
	calculateExternalForce();

	// update cloth
	switch (m_integration_method)
	{
	case INTEGRATION_EXPLICIT_EULER:
		//TODO
		break;
	case INTEGRATION_EXPLICIT_SYMPLECTIC:
		//TODO
		break;
	case INTEGRATION_IMPLICIT_EULER_BARAFF_WITKIN:
		//TODO
		break;
	case INTEGRATION_GRADIENT_DESCENT:
	case INTEGRATION_NEWTON_DESCENT:
	case INTEGRATION_NEWTON_DESCENT_PCG:
	case INTEGRATION_LOCAL_GLOBAL:
		//TODO
		break;
	case INTEGRATION_PBD:
		//TODO
		break;
	}

	// Add collision detection here using pos_next;
	VectorX penetration = collisionDetection(m_mesh->m_current_positions);
	m_mesh->m_current_positions -= penetration;

	// update velocity and damp
	dampVelocity();
}

void Simulation::DrawConstraints(const VBO& vbos)
{
	for (std::vector<Constraint*>::iterator it = m_constraints.begin(); it != m_constraints.end(); ++it)
	{
		(*it)->Draw(vbos);
	}
}

ScalarType Simulation::TryToSelectAttachmentConstraint(const EigenVector3& p0, const EigenVector3& dir)
{
	ScalarType ray_point_dist;
	ScalarType min_dist = 100.0;
	AttachmentConstraint* best_candidate = NULL;

	bool current_state_on = false;
	for (std::vector<Constraint*>::iterator c = m_constraints.begin(); c != m_constraints.end(); ++c)
	{
		AttachmentConstraint* ac;
		if (ac = dynamic_cast<AttachmentConstraint*>(*c)) // is attachment constraint
		{
			ray_point_dist = ((ac->GetFixedPoint()-p0).cross(dir)).norm();
			if (ray_point_dist < min_dist)
			{
				min_dist = ray_point_dist;
				best_candidate = ac;
			}
		}
	}
	// exit if no one fits
	if (min_dist > DEFAULT_SELECTION_RADIUS)
	{
		UnselectAttachmentConstraint();

		return -1;
	}
	else
	{
		SelectAtttachmentConstraint(best_candidate);
		EigenVector3 fixed_point_temp = m_mesh->m_current_positions.block_vector(m_selected_attachment_constraint->GetConstrainedVertexIndex());

		return (fixed_point_temp-p0).dot(dir); // this is m_cached_projection_plane_distance
	}
}

bool Simulation::TryToToggleAttachmentConstraint(const EigenVector3& p0, const EigenVector3& dir)
{
	EigenVector3 p1;

	ScalarType ray_point_dist;
	ScalarType min_dist = 100.0;
	unsigned int best_candidate = 0;
	// first pass: choose nearest point
	for (unsigned int i = 0; i != m_mesh->m_vertices_number; i++)
	{
		p1 = m_mesh->m_current_positions.block_vector(i);
		
		ray_point_dist = ((p1-p0).cross(dir)).norm();
		if (ray_point_dist < min_dist)
		{
			min_dist = ray_point_dist;
			best_candidate = i;
		}
	}
	for (std::vector<Constraint*>::iterator c = m_constraints.begin(); c != m_constraints.end(); ++c)
	{
		AttachmentConstraint* ac;
		if (ac = dynamic_cast<AttachmentConstraint*>(*c)) // is attachment constraint
		{
			ray_point_dist = ((ac->GetFixedPoint()-p0).cross(dir)).norm();
			if (ray_point_dist < min_dist)
			{
				min_dist = ray_point_dist;
				best_candidate = ac->GetConstrainedVertexIndex();
			}
		}
	}
	// exit if no one fits
	if (min_dist > DEFAULT_SELECTION_RADIUS)
	{
		return false;
	}
	// second pass: toggle that point's fixed position constraint
	bool current_state_on = false;
	for (std::vector<Constraint*>::iterator c = m_constraints.begin(); c != m_constraints.end(); ++c)
	{
		AttachmentConstraint* ac;
		if (ac = dynamic_cast<AttachmentConstraint*>(*c)) // is attachment constraint
		{
			if (ac->GetConstrainedVertexIndex() == best_candidate)
			{
				current_state_on = true;
				m_constraints.erase(c);
				delete ac;
				m_mesh->m_expanded_system_dimension-=3;
				m_mesh->m_expanded_system_dimension_1d-=1;
				break;
			}
		}
	}
	if (!current_state_on)
	{
		AddAttachmentConstraint(best_candidate);
	}

	return true;
}

void Simulation::SelectAtttachmentConstraint(AttachmentConstraint* ac)
{
	m_selected_attachment_constraint = ac;
	m_selected_attachment_constraint->Select();
}

void Simulation::UnselectAttachmentConstraint()
{
	if (m_selected_attachment_constraint)
	{
		m_selected_attachment_constraint->UnSelect();
	}
	m_selected_attachment_constraint = NULL;
}

void Simulation::AddAttachmentConstraint(unsigned int vertex_index)
{
	AttachmentConstraint* ac = new AttachmentConstraint(&m_stiffness_attachment, vertex_index, m_mesh->m_current_positions.block_vector(vertex_index));
	m_constraints.push_back(ac);
	m_mesh->m_expanded_system_dimension+=3;
	m_mesh->m_expanded_system_dimension_1d+=1;
}

void Simulation::MoveSelectedAttachmentConstraintTo(const EigenVector3& target)
{
	if (m_selected_attachment_constraint)
		m_selected_attachment_constraint->SetFixedPoint(target);
}

void Simulation::SaveAttachmentConstraint(const char* filename)
{
	std::ofstream outfile;
	outfile.open(filename, std::ifstream::out);
	if (outfile.is_open())
	{
		int existing_vertices = 0;
		for (std::vector<Constraint*>::iterator c = m_constraints.begin(); c != m_constraints.end(); ++c)
		{
			(*c)->WriteToFileOBJHead(outfile);
		}
		outfile << std::endl;
		for (std::vector<Constraint*>::iterator c = m_constraints.begin(); c != m_constraints.end(); ++c)
		{
			(*c)->WriteToFileOBJ(outfile, existing_vertices);
		}

		outfile.close();
	}
}

void Simulation::LoadAttachmentConstraint(const char* filename)
{
	// clear current attachement constraints
	for (std::vector<Constraint*>::iterator& c = m_constraints.begin(); c != m_constraints.end(); )
	{
		AttachmentConstraint* ac;
		if (ac = dynamic_cast<AttachmentConstraint*>(*c)) // is attachment constraint
		{
			c = m_constraints.erase(c);
			delete ac;
			m_mesh->m_expanded_system_dimension-=3;
			m_mesh->m_expanded_system_dimension_1d-=1;
		}
		else
		{
			c++;
		}
	}

	// read from file
	std::ifstream infile;
	infile.open(filename, std::ifstream::in);
	char ignore[256];
	if (infile.is_open())
	{
		while(!infile.eof())
		{
			int id;
			EigenVector3 p;
			if (infile >> ignore >> id >> p[0] >> p[1] >> p[2])
			{
				if (strcmp(ignore, "v") == 0)
					break;
				AttachmentConstraint* ac = new AttachmentConstraint(&m_stiffness_attachment, id, p);
				m_constraints.push_back(ac);
				m_mesh->m_expanded_system_dimension+=3;
				m_mesh->m_expanded_system_dimension_1d+=1;
			}
			else
				break;
		}

		infile.close();
	}

}

void Simulation::clearConstraints()
{
	for (unsigned int i = 0; i < m_constraints.size(); ++i)
	{
		delete m_constraints[i];
	}
	m_constraints.clear();
}

void Simulation::setupConstraints()
{
	clearConstraints();

	switch(m_mesh->m_mesh_type)
	{
	case MESH_TYPE_CLOTH:
		// procedurally generate constraints including to attachment constraints
		{
			// generate stretch constraints. assign a stretch constraint for each edge.
			EigenVector3 p1, p2;
			for(std::vector<Edge>::iterator e = m_mesh->m_edge_list.begin(); e != m_mesh->m_edge_list.end(); ++e)
			{
				p1 = m_mesh->m_current_positions.block_vector(e->m_v1);
				p2 = m_mesh->m_current_positions.block_vector(e->m_v2);
				SpringConstraint* c = new SpringConstraint(&m_stiffness_stretch, e->m_v1, e->m_v2, (p1-p2).norm());
				m_constraints.push_back(c);
				m_mesh->m_expanded_system_dimension+=6;
				m_mesh->m_expanded_system_dimension_1d+=2;
			}

			// generate bending constraints. naive way
			unsigned int i, k;
			for(i = 0; i < m_mesh->m_dim[0]; ++i)
			{
				for(k = 0; k < m_mesh->m_dim[1]; ++k)
				{
					unsigned int index_self = m_mesh->m_dim[1] * i + k;
					p1 = m_mesh->m_current_positions.block_vector(index_self);
					if (i+2 < m_mesh->m_dim[0])
					{
						unsigned int index_row_1 = m_mesh->m_dim[1] * (i + 2) + k;
						p2 = m_mesh->m_current_positions.block_vector(index_row_1);
						SpringConstraint* c = new SpringConstraint(&m_stiffness_bending, index_self, index_row_1, (p1-p2).norm());
						m_constraints.push_back(c);
						m_mesh->m_expanded_system_dimension+=6;
						m_mesh->m_expanded_system_dimension_1d+=2;
					}
					if (k+2 < m_mesh->m_dim[1])
					{
						unsigned int index_column_1 = m_mesh->m_dim[1] * i + k + 2;
						p2 = m_mesh->m_current_positions.block_vector(index_column_1);
						SpringConstraint* c = new SpringConstraint(&m_stiffness_bending, index_self, index_column_1, (p1-p2).norm());
						m_constraints.push_back(c);
						m_mesh->m_expanded_system_dimension+=6;
						m_mesh->m_expanded_system_dimension_1d+=2;
					}
				}
			}

			// generating attachment constraints.
			AddAttachmentConstraint(0);
			AddAttachmentConstraint(m_mesh->m_dim[1]*(m_mesh->m_dim[0]-1));
		}
		break;
	case MESH_TYPE_TET:
		{
			VectorX& x = m_mesh->m_current_positions;
			TetMesh* tet_mesh = dynamic_cast<TetMesh*>(m_mesh);
			for (unsigned int i = 0; i < tet_mesh->m_loaded_mesh->m_tets.size(); ++i)
			{
				MeshLoader::Tet& tet = tet_mesh->m_loaded_mesh->m_tets[i];
				TetConstraint *c = new TetConstraint(&m_stiffness_stretch, tet.id1, tet.id2, tet.id3, tet.id4, x);
				m_constraints.push_back(c);
				m_mesh->m_expanded_system_dimension+=9;
				m_mesh->m_expanded_system_dimension_1d+=3;
			}
		}
		break;
	}
}

void Simulation::dampVelocity()
{
	// TODO: post processing damping 
}

void Simulation::calculateInertiaY()
{
	m_inertia_y = m_mesh->m_current_positions + m_mesh->m_current_velocities * m_h;
}

void Simulation::calculateExternalForce()
{
	m_external_force.resize(m_mesh->m_system_dimension);
	m_external_force.setZero();

	// gravity
	for (unsigned int i = 0; i < m_mesh->m_vertices_number; ++i)
	{
		m_external_force[3*i+1] += -m_gravity_constant;
	}

	m_external_force = m_mesh->m_mass_matrix * m_external_force;
}

VectorX Simulation::collisionDetection(const VectorX x)
{
	// Naive implementation of collision detection
	VectorX penetration(m_mesh->m_system_dimension);
	penetration.setZero();
	EigenVector3 normal;
	ScalarType dist;

	for (unsigned int i = 0; i < m_mesh->m_vertices_number; ++i)
	{
		EigenVector3 xi = x.block_vector(i);

		if (m_scene->StaticIntersectionTest(xi, normal, dist))
		{
			penetration.block_vector(i) += (dist) * normal;
		}
	}

	return penetration;
}