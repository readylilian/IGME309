#include "AppClass.h"
void Application::InitVariables(void)
{
	////Change this to your name and email
	m_sProgrammer = "Alberto Bobadilla - labigm@rit.edu";
	vector3 v3Position(0.0f, 0.0f, 10.0f);
	vector3 v3Target = ZERO_V3;
	vector3 v3Upward = AXIS_Y;
	m_pCameraMngr->SetPositionTargetAndUpward(v3Position, v3Target, v3Upward);

	m_pModelMngr->LoadModel("Walleye.obj");
}
void Application::Update(void)
{
	//Update the system so it knows how much time has passed since the last call
	m_pSystem->Update();

	//Is the arcball active?
	ArcBall();

	//Is the first person camera active?
	CameraRotation();

	//Update Entity Manager
	m_pEntityMngr->Update();

	static float fDelta = 5.0f;
	//Swap any of these for fDelta to see the effect

	//Where the camera is standing
	vector3 v3Postition(0.0f, 0.0f, 10.0f);
	
	//Where the camera is looking
	vector3 v3Target(0.0f, 0.0f, 0.0f);

	//Sets where UP is, so if y = -1, then everything is upside down
	vector3 v3Upwards(1.0f, 0.0f, 0.0f);
	
	m_pCameraMngr->SetUpward(v3Upwards);
	m_pCameraMngr->SetTarget(v3Target);
	m_pCameraMngr->SetPosition(v3Postition);
	fDelta += m_pSystem->GetDeltaTime(0);

	//std::cout << fDelta << std::endl;

	m_pModelMngr->AddModelToRenderList("Walleye.obj", ToMatrix4(m_qArcBall), BTX::eBTX_RENDER::RENDER_WIRE | BTX::eBTX_RENDER::RENDER_SOLID);
	//Add objects to render list
	m_pEntityMngr->AddEntityToRenderList(-1, true);
}
void Application::Display(void)
{
	// Clear the screen
	ClearScreen();

	// draw a skybox
	m_pModelMngr->AddSkyboxToRenderList();

	//render list call
	m_uRenderCallCount = m_pModelMngr->Render();

	//clear the render list
	m_pModelMngr->ClearRenderList();

	//draw gui
	DrawGUI();
	
	//end the current frame (internally swaps the front and back buffers)
	m_pWindow->display();
}
void Application::Release(void)
{
	//release GUI
	ShutdownGUI();
}