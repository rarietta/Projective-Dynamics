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

#pragma warning( disable : 4244)
#include <omp.h>

#include <iostream>
#include <string>

//----------Headers--------------//
#include "global_headers.h"
#include "math_headers.h"
#include "openGL_headers.h"
//----------Framework--------------//
#include "fps.h"
#include "timer_wrapper.h"
#include "stb_image_write.h"
#include "glsl_wrapper.h"
#include "AntTweakBar.h"
#include "anttweakbar_wrapper.h"
#include "camera.h"
#include "scene.h"
//----------Core--------------//
#include "mesh.h"
#include "simulation.h"

//----------Project Key Globals--------------//
AntTweakBarWrapper* g_config_bar;
Camera* g_camera;
RenderWrapper* g_renderer;
Scene* g_scene;
Mesh* g_mesh;
Simulation * g_simulation;
TimerWrapper g_global_timer;
int g_interval = 100;
ScalarType total_time = 0;

//----------Global Parameters----------------//
int g_screen_width = DEFAULT_SCREEN_WIDTH;
int g_screen_height = DEFAULT_SCREEN_HEIGHT;

//----------State Control--------------------//
bool g_only_show_sim = false;
bool g_record = false;
bool g_pause = true;
bool g_show_mesh = true;
bool g_show_wireframe = false;
int  g_wireframe_linewidth = 1;
bool g_show_texture = false;
bool g_texture_load_succeed = false;

//----------Mouse Control--------------------//
int g_mouse_old_x, g_mouse_old_y;
int g_mouse_wheel_pos;
unsigned char g_button_mask = 0x00;

//----------Frame Rate/Frame Number----------//
mmc::FpsTracker g_fps_tracker;
int g_max_fps = 30;
int g_timestep = 1000 / g_max_fps;

//----------Recording Related----------------//
bool g_recording_limit = false;
int g_current_frame = 0;
int g_total_frame = 0;

//----------glut function handlers-----------//
void resize(int, int);
void timeout(int);
void display(void);
void key_press(unsigned char, int, int);
void mouse_click(int, int, int, int);
void mouse_motion(int, int);
void mouse_wheel(int, int, int, int);
void mouse_over(int, int);

//----------anttweakbar handlers----------//
void TW_CALL reset_simulation(void*);
void TW_CALL step_through(void*);
void TW_CALL reset_camera(void*);

//----------other utility functions----------//
void init(void);
void cleanup(void);
void draw_overlay(void);
void grab_screen(void);
void grab_screen(char* filename);

int main(int argc, char ** argv)
{
	// gl init
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA);
	glutCreateWindow("Projective Dynamics T.L.");
	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LEQUAL);
	glutInitWindowSize(g_screen_width, g_screen_height);
	glViewport(0, 0, g_screen_width, g_screen_height);

	// user init
	init();
	glutReshapeWindow(g_screen_width, g_screen_height);

	// bind function callbacks
	glutDisplayFunc(display);
	glutTimerFunc(g_timestep, timeout, g_timestep);
	glutReshapeFunc(resize);
	glutKeyboardFunc(key_press);
	glutMouseFunc(mouse_click);
	glutMotionFunc(mouse_motion);
	glutPassiveMotionFunc(mouse_over);
	glutMouseWheelFunc(mouse_wheel);
	glutCloseFunc(cleanup);
	glutIdleFunc(display);

	glutMainLoop();

	return 0;
}

void resize(int width, int height) {
	g_screen_width = width;
	g_screen_height = height;
	//set the viewport, more boilerplate
	glViewport(0, 0, width, height);
	g_camera->ResizeWindow(width, height);
	g_config_bar->ChangeTwBarWindowSize(g_screen_width, g_screen_height);

	glutPostRedisplay();
}

void timeout(int value)
{
	glutTimerFunc(g_timestep, timeout, g_timestep);
	// keep track of time
	g_fps_tracker.timestamp();

	// ant tweak bar update
	int atb_feed_back = g_config_bar->Update();
	if (atb_feed_back&ATB_RESHAPE_WINDOW)
	{
		glutReshapeWindow(g_screen_width, g_screen_height);
	}
	if (atb_feed_back&(ATB_CHANGE_STIFFNESS|ATB_CHANGE_TIME_STEP))
	{
		g_simulation->SetReprefactorFlag();
	}
	if (atb_feed_back&(ATB_INIT_MATLAB))
	{

	}

	if (g_recording_limit && g_current_frame > g_total_frame)
	{
		g_pause = true;
	}

	// simulation update
	if (!g_pause) 
	{
		// update animation
		g_simulation->UpdateAnimation(g_current_frame);

		// update mesh
		g_simulation->Update();

		// grab screen
		if (g_record) 
		{
			char cap_filename[256];
			sprintf_s(cap_filename, 256, "output/ScreenCap%04d.png", g_current_frame);
			grab_screen(cap_filename);


			// TODO: If you want to export everything to Maya, this is what you need:
			//char mesh_filename[256];
			//sprintf_s(mesh_filename, 256, "output/Mesh%04d.obj", g_current_frame);
			//g_mesh->ExportToOBJ(mesh_filename);
			//char handle_filename[256];
			//sprintf_s(handle_filename, 256, "output/Handle%04d.obj", g_current_frame);
			//g_simulation->SaveAttachmentConstraint(handle_filename);
		}

		g_current_frame ++;
	}

	glutPostRedisplay();
}

void display() {

	//Always and only do this at the start of a frame, it wipes the slate clean
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

	// aim camera
	g_renderer->SetCameraModelview(g_camera->GetViewMatrix());
	g_renderer->SetCameraProjection(g_camera->GetProjectionMatrix());

	// Draw world and cloth (using programmable shaders)
	g_renderer->ActivateShaderprog();
	g_scene->Draw(g_renderer->getVBO());
	if (g_show_mesh)
	{
		g_mesh->Draw(g_renderer->getVBO(), g_show_texture & g_texture_load_succeed);
	}
	if (g_show_wireframe)
	{
		g_mesh->DrawWireFrame(g_renderer->getVBO(), g_wireframe_linewidth);
	}
	g_simulation->DrawConstraints(g_renderer->getVBO());
	g_renderer->DeactivateShaderprog();

	if (!g_only_show_sim)
	{
		// Draw axis
		g_camera->DrawAxis();

		// Draw overlay
		draw_overlay();
	}

	// Draw tweak bar
	g_config_bar->Draw();

	glutSwapBuffers();
}

void key_press(unsigned char key, int x, int y) {
	if (!TwEventKeyboardGLUT(key, x, y))
	{
		switch(key) {
		case 32:
			g_pause = !g_pause;
			break;
		case 'r':
		case 'R':
			g_record = !g_record;
			break;
		case '1':
			g_show_wireframe = true;
			g_show_mesh = false;
			break;
		case '2':
			g_show_wireframe = true;
			g_show_mesh = true;
			break;
		case '3':
			g_show_wireframe = false;
			g_show_mesh = true;
			break;
		case 't':
		case 'T':
			g_show_texture = !g_show_texture;
			break;
		case 'p':
		case 'P':
			step_through(NULL);
			break;
		case 'q': 
		case 'Q':
		case 27: // ascii code of esc key
			cleanup();
			exit(EXIT_SUCCESS);
			break;
		case 'h':
		case 'H':
			if (g_only_show_sim)
			{
				g_only_show_sim = false;
				g_config_bar->Show();
			}
			else
			{
				g_only_show_sim = true;
				g_config_bar->Hide();
			}
			break;
		case 's':
		case 'S':
			g_camera->SaveCamera();
			g_mesh->ExportToOBJ(DEFAULT_CONFIG_OBJ_FILE);
			g_simulation->SaveAttachmentConstraint(DEFAULT_CONFIG_HANDLE_FILE);
			break;
		case 'l':
		case 'L':
			if (g_mesh->ImportFromOBJ(DEFAULT_CONFIG_OBJ_FILE))
			{
				g_simulation->LoadAttachmentConstraint(DEFAULT_CONFIG_HANDLE_FILE);
			}
			g_camera->LoadCamera();
			glutPostRedisplay();
			break;
		case 'g':
		case 'G':
			grab_screen(DEFAULT_SCREEN_SHOT_FILE);
			g_mesh->ExportToOBJ(DEFAULT_OUTPUT_CLOTH_OBJ_FILE);
			g_simulation->SaveAttachmentConstraint(DEFAULT_OUTPUT_ATTACHMENT_OBJ_FILE);
			break;
		case 'f':
		case 'F':
			g_camera->Lookat(g_mesh);
			break;
		}
	}

	glutPostRedisplay();
}

void mouse_click(int button, int state, int x, int y)
{
	if (!TwEventMouseButtonGLUT(button, state, x, y))
	{
		switch(state)
		{
		case GLUT_DOWN:
			if (glutGetModifiers() == GLUT_ACTIVE_ALT)
			{
				// left: 0. right: 2. middle: 1.
				g_button_mask |= 0x01 << button;
				g_mouse_old_x = x;
				g_mouse_old_y = y;
			}
			else if (glutGetModifiers() == GLUT_ACTIVE_CTRL)
			{
				// ctrl: 3
				g_button_mask |= 0x01 << 3;
				g_mouse_old_x = x;
				g_mouse_old_y = y;
			}
			else
			{
				if (g_simulation->TryToToggleAttachmentConstraint(GLM2Eigen(g_camera->GetCameraPosition()), GLM2Eigen(g_camera->GetRaycastDirection(x, y))))
				{ // hit something
					g_simulation->SetReprefactorFlag();
				}
			}
		   break;
		case GLUT_UP:
			if (glutGetModifiers() == GLUT_ACTIVE_CTRL)
			{// special case for ctrl
				button = 3;
			}

			g_simulation->UnselectAttachmentConstraint();

			unsigned char mask_not = ~g_button_mask;
			mask_not |= 0x01 << button;
			g_button_mask = ~mask_not;
			break;
		}
	}
}

void mouse_motion(int x, int y)
{
	if (!TwEventMouseMotionGLUT(x, y))
	{
		float dx, dy;
		dx = (float)(x - g_mouse_old_x);
		dy = (float)(y - g_mouse_old_y);

		if (g_button_mask & 0x01) 
		{// left button
			g_camera->MouseChangeHeadPitch(0.2f, dx, dy);
		} 
		else if (g_button_mask & 0x02)
		{// middle button
			g_camera->MouseChangeLookat(0.01f, dx, dy);
		}
		else if (g_button_mask & 0x04) 
		{// right button
			g_camera->MouseChangeDistance(0.05f, dx, dy);
		}
		else if (g_button_mask & 0x08)
		{// ctrl + button
			g_simulation->MoveSelectedAttachmentConstraintTo(GLM2Eigen(g_camera->GetCurrentTargetPoint(x, y)));
		}

		g_mouse_old_x = x;
		g_mouse_old_y = y;
	}
}

void mouse_wheel(int button, int dir, int x, int y)
{
	if (!TwMouseWheel(g_mouse_wheel_pos+=dir))
	{
		g_camera->MouseChangeDistance(1.0f, 0, (ScalarType)(dir));
	}
}

void mouse_over(int x, int y)
{
	if (!TwEventMouseMotionGLUT(x, y))
	{
		if (glutGetModifiers() == GLUT_ACTIVE_CTRL)
		{// ctrl + mouse hover
			ScalarType projection_plane_distance = g_simulation->TryToSelectAttachmentConstraint(GLM2Eigen(g_camera->GetCameraPosition()), GLM2Eigen(g_camera->GetRaycastDirection(x, y)));
			if (projection_plane_distance > 0)
			{
				g_camera->SetProjectionPlaneDistance(projection_plane_distance);
			}
		}
	}
}

void init()
{
	// glew init
	fprintf(stdout, "Initializing glew...\n");
	glewInit();
	if (!glewIsSupported( "GL_VERSION_2_0 " 
		"GL_ARB_pixel_buffer_object"
		)) {
			std::cerr << "ERROR: Support for necessary OpenGL extensions missing." << std::endl;
			exit(EXIT_FAILURE);
	}

	// config init
	fprintf(stdout, "Initializing AntTweakBar...\n");
	g_config_bar = new AntTweakBarWrapper();
	g_config_bar->ChangeTwBarWindowSize(g_screen_width, g_screen_height);

	// render wrapper init
	fprintf(stdout, "Initializing render wrapper...\n");
	g_renderer = new RenderWrapper();
	g_renderer->InitShader(DEFAULT_VERT_SHADER_FILE, DEFAULT_FRAG_SHADER_FILE);
	g_texture_load_succeed = g_renderer->InitTexture(DEFAULT_TEXTURE_FILE);

	// camera init
	fprintf(stdout, "Initializing camera...\n");
	g_camera = new Camera();

	// scene init
	fprintf(stdout, "Initializing scene...\n");
	g_scene = new Scene(DEFAULT_SCENE_FILE);

	// mesh init
	fprintf(stdout, "Initializing mesh...\n");
	g_mesh = new Mesh();

	// simulation init
	fprintf(stdout, "Initializing simulation...\n");
	g_simulation = new Simulation();

	// load or get default value
	g_config_bar->LoadSettings();

	reset_camera(NULL);
	reset_simulation(NULL);
}

void cleanup() // clean up in a reverse order
{
	if (g_scene)
		delete g_scene;
	if (g_camera)
		delete g_camera;
	if (g_renderer)
	{
		g_renderer->CleanupShader();
		delete g_renderer;
	}
	if (g_config_bar)
	{
		delete g_config_bar;
	}
}

void TW_CALL reset_simulation(void*)
{
	// save current setting before reset
	AntTweakBarWrapper::SaveSettings(g_config_bar);

	// reset frame#
	g_current_frame = 0;
	g_pause = true;

	switch(g_mesh->GetMeshType())
	{
	case MESH_TYPE_CLOTH:
		delete g_mesh;
		g_mesh = new ClothMesh();
		break;
	case MESH_TYPE_TET:
		delete g_mesh;
		g_mesh = new TetMesh();
		break;
	}
	g_config_bar->LoadSettings();
	g_mesh->Reset();

	// reset simulation
	g_simulation->SetMesh(g_mesh);
	g_simulation->SetScene(g_scene);

	g_simulation->Reset();

	// reset config, (config bar is recommended to reset last)
	g_config_bar->Reset();
}

void TW_CALL reset_camera(void*)
{
	// reset camera
	g_camera->Reset(g_screen_width, g_screen_height);
}

void TW_CALL step_through(void*)
{
	if(!g_pause)
	{
		g_pause = true;
	}

	// enable step mode
	g_simulation->SetStepMode(true);
	// update cloth
	g_simulation->Update();
	// disable step mode
	g_simulation->SetStepMode(false);

	g_current_frame++;
}

void grab_screen(void)
{
	char anim_filename[256];
	sprintf_s(anim_filename, 256, "output/Simulation%04d.png", g_current_frame);
	grab_screen(anim_filename);
}

void grab_screen(char* filename)
{
	unsigned char* bitmapData = new unsigned char[3 * g_screen_width * g_screen_height];

	for (int i=0; i < g_screen_height; i++) 
	{
		glReadPixels(0, i, g_screen_width, 1, GL_RGB, GL_UNSIGNED_BYTE, 
			bitmapData + (g_screen_width * 3 * ((g_screen_height - 1) - i)));
	}

	stbi_write_png(filename, g_screen_width, g_screen_height, 3, bitmapData, g_screen_width * 3);

	delete [] bitmapData;
}

void draw_overlay()
{
	// Draw Overlay
	glColor4d(0.0, 0.0, 0.0, 1.0);
	glPushAttrib(GL_LIGHTING_BIT);
	glDisable(GL_LIGHTING);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluOrtho2D(0.0, 1.0, 0.0, 1.0);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glRasterPos2d(0.03, 0.01);

	char overlay_char_from_mesh[255] = ""; 
	g_mesh->GetMeshInfo(overlay_char_from_mesh);
	//g_simulation->GetOverlayChar(overlay_char_from_simulation);

	char info[1024];
	sprintf_s(info, "FPS: %3.1f | Frame#: %d | %s", g_fps_tracker.fpsAverage(), g_current_frame, overlay_char_from_mesh);

	for (unsigned int i = 0; i < strlen(info); i++)
	{
		glutBitmapCharacter(GLUT_BITMAP_HELVETICA_12, info[i]);
	}

	glPopAttrib();
}