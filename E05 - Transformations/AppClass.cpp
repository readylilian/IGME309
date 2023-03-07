#include "AppClass.h"
void Application::InitVariables(void)
{
	////Change this to your name and email
	m_sProgrammer = "Alberto Bobadilla - labigm@rit.edu";
	vector3 v3Position(0.0f, 0.0f, 10.0f);
	vector3 v3Target = ZERO_V3;
	vector3 v3Upward = AXIS_Y;
	m_pCameraMngr->SetPositionTargetAndUpward(v3Position, v3Target, v3Upward);
	
	std::vector<MyMesh*> m_pMeshList;
	//Allocate the memory for the Meshes
	m_pMesh = new MyMesh();
	m_pMesh->GenerateCube(1.0f, C_BLACK);
	//pMesh = new MyMesh();
	
	
		
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

	//Add objects to render list
	m_pEntityMngr->AddEntityToRenderList(-1, true);
}
void Application::Display(void)
{
	// Clear the screen
	ClearScreen();

	//Calculate the model, view and projection matrix
	matrix4 m4Projection = m_pCameraMngr->GetProjectionMatrix();
	matrix4 m4View = m_pCameraMngr->GetViewMatrix();
	
	static bool Left = false;
	static float fDisp = 0.0f;
	matrix4 m4Model = glm::translate(IDENTITY_M4, vector3(fDisp, 0.0f,0.0f));
	
	//Row + 4
	m_pMesh->Render(m4Projection, m4View, glm::translate(m4Model, vector3(-3.0f, 4.0f, 0.0f)));
	m_pMesh->Render(m4Projection, m4View, glm::translate(m4Model, vector3(3.0f, 4.0f, 0.0f)));
	//Row + 3
	m_pMesh->Render(m4Projection, m4View, glm::translate(m4Model, vector3(-2.0f, 3.0f, 0.0f)));
	m_pMesh->Render(m4Projection, m4View, glm::translate(m4Model, vector3(2.0f, 3.0f, 0.0f)));
	//Row + 2

	m_pMesh->Render(m4Projection, m4View, glm::translate(m4Model, vector3(-3.0f, 2.0f, 0.0f)));
	m_pMesh->Render(m4Projection, m4View, glm::translate(m4Model, vector3(-2.0f, 2.0f, 0.0f)));
	m_pMesh->Render(m4Projection, m4View, glm::translate(m4Model, vector3(-1.0f, 2.0f, 0.0f)));
	m_pMesh->Render(m4Projection, m4View, glm::translate(m4Model, vector3(0.0f, 2.0f, 0.0f)));
	m_pMesh->Render(m4Projection, m4View, glm::translate(m4Model, vector3(1.0f, 2.0f, 0.0f)));
	m_pMesh->Render(m4Projection, m4View, glm::translate(m4Model, vector3(2.0f, 2.0f, 0.0f)));
	m_pMesh->Render(m4Projection, m4View, glm::translate(m4Model, vector3(3.0f, 2.0f, 0.0f)));

	//Row + 1
	m_pMesh->Render(m4Projection, m4View, glm::translate(m4Model, vector3(-4.0f, 1.0f, 0.0f)));
	m_pMesh->Render(m4Projection, m4View, glm::translate(m4Model, vector3(-3.0f, 1.0f, 0.0f)));
	m_pMesh->Render(m4Projection, m4View, glm::translate(m4Model, vector3(-1.0f, 1.0f, 0.0f)));
	m_pMesh->Render(m4Projection, m4View, glm::translate(m4Model, vector3(0.0f, 1.0f, 0.0f)));
	m_pMesh->Render(m4Projection, m4View, glm::translate(m4Model, vector3(1.0f, 1.0f, 0.0f)));
	m_pMesh->Render(m4Projection, m4View, glm::translate(m4Model, vector3(3.0f, 1.0f, 0.0f)));
	m_pMesh->Render(m4Projection, m4View, glm::translate(m4Model, vector3(4.0f, 1.0f, 0.0f)));

	//Center row
	m_pMesh->Render(m4Projection, m4View, m4Model);
	m_pMesh->Render(m4Projection, m4View, glm::translate(m4Model, vector3(1.0f,0.0f,0.0f)));
	m_pMesh->Render(m4Projection, m4View, glm::translate(m4Model, vector3(2.0f, 0.0f, 0.0f)));
	m_pMesh->Render(m4Projection, m4View, glm::translate(m4Model, vector3(3.0f, 0.0f, 0.0f)));
	m_pMesh->Render(m4Projection, m4View, glm::translate(m4Model, vector3(4.0f, 0.0f, 0.0f)));
	m_pMesh->Render(m4Projection, m4View, glm::translate(m4Model, vector3(5.0f, 0.0f, 0.0f)));

	m_pMesh->Render(m4Projection, m4View, glm::translate(m4Model, vector3(-1.0f, 0.0f, 0.0f)));
	m_pMesh->Render(m4Projection, m4View, glm::translate(m4Model, vector3(-2.0f, 0.0f, 0.0f)));
	m_pMesh->Render(m4Projection, m4View, glm::translate(m4Model, vector3(-3.0f, 0.0f, 0.0f)));
	m_pMesh->Render(m4Projection, m4View, glm::translate(m4Model, vector3(-4.0f, 0.0f, 0.0f)));
	m_pMesh->Render(m4Projection, m4View, glm::translate(m4Model, vector3(-5.0f, 0.0f, 0.0f)));
	
	//Row - 1
	m_pMesh->Render(m4Projection, m4View, glm::translate(m4Model, vector3(0.0f, -1.0f, 0.0f)));
	m_pMesh->Render(m4Projection, m4View, glm::translate(m4Model, vector3(1.0f, -1.0f, 0.0f)));
	m_pMesh->Render(m4Projection, m4View, glm::translate(m4Model, vector3(2.0f, -1.0f, 0.0f)));
	m_pMesh->Render(m4Projection, m4View, glm::translate(m4Model, vector3(3.0f, -1.0f, 0.0f)));
	m_pMesh->Render(m4Projection, m4View, glm::translate(m4Model, vector3(5.0f, -1.0f, 0.0f)));

	m_pMesh->Render(m4Projection, m4View, glm::translate(m4Model, vector3(-1.0f, -1.0f, 0.0f)));
	m_pMesh->Render(m4Projection, m4View, glm::translate(m4Model, vector3(-2.0f, -1.0f, 0.0f)));
	m_pMesh->Render(m4Projection, m4View, glm::translate(m4Model, vector3(-3.0f, -1.0f, 0.0f)));
	m_pMesh->Render(m4Projection, m4View, glm::translate(m4Model, vector3(-5.0f, -1.0f, 0.0f)));

	//Row -2
	m_pMesh->Render(m4Projection, m4View, glm::translate(m4Model, vector3(3.0f, -2.0f, 0.0f)));
	m_pMesh->Render(m4Projection, m4View, glm::translate(m4Model, vector3(5.0f, -2.0f, 0.0f)));
	m_pMesh->Render(m4Projection, m4View, glm::translate(m4Model, vector3(-3.0f, -2.0f, 0.0f)));
	m_pMesh->Render(m4Projection, m4View, glm::translate(m4Model, vector3(-5.0f, -2.0f, 0.0f)));

	//Row - 3
	m_pMesh->Render(m4Projection, m4View, glm::translate(m4Model, vector3(1.0f, -3.0f, 0.0f)));
	m_pMesh->Render(m4Projection, m4View, glm::translate(m4Model, vector3(2.0f, -3.0f, 0.0f)));
	m_pMesh->Render(m4Projection, m4View, glm::translate(m4Model, vector3(-1.0f, -3.0f, 0.0f)));
	m_pMesh->Render(m4Projection, m4View, glm::translate(m4Model, vector3(-2.0f, -3.0f, 0.0f)));
	
	if (Left == false) {
		fDisp += .05;
		if (fDisp > 5)
		{
			Left = true;
		}
	}
	if (Left == true) {
		fDisp -= .05;
		if (fDisp < -5)
		{
			Left = false;
		}
	}

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
	//Release meshes
	SafeDelete(m_pMesh);

	//release GUI
	ShutdownGUI();
}