/* -*-c++-*- $Id: Version,v 1.2 2004/04/20 12:26:04 andersb Exp $ */
/**
* OsgHaptics - OpenSceneGraph Haptic Library
* Copyright (C) 2006 VRlab, Umeå University
*
* This library is free software; you can redistribute it and/or
* modify it under the terms of the GNU Lesser General Public
* License as published by the Free Software Foundation; either
* version 2.1 of the License, or (at your option) any later version.
*
* This library is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
* Lesser General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public
* License along with this library; if not, write to the Free Software
* Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA.
*/


// osgHaptics.cpp : Defines the entry point for the DLL application.
//

#include <osgHaptics/osgHaptics.h>


#ifdef _WIN32

#include <osgHaptics/export.h>
#include <windows.h>

BOOL APIENTRY DllMain( HANDLE hModule, 
											DWORD  ul_reason_for_call, 
											LPVOID lpReserved
											)
{
	switch (ul_reason_for_call)
	{
	case DLL_PROCESS_ATTACH:
	case DLL_THREAD_ATTACH:
	case DLL_THREAD_DETACH:
	case DLL_PROCESS_DETACH:
		break;
	}
	return TRUE;
}

#endif

using namespace osgHaptics;

void osgHaptics::prepareHapticCamera(osg::Camera *camera, HapticDevice *device, osg::Node *scene) 
{

	camera->setPreDrawCallback(new HapticDevicePreRenderCallback(device));
	camera->setPostDrawCallback(new HapticDevicePostRenderCallback(device));

	if (0 && scene) {
		osg::BoundingSphere bs = scene->getBound();
		osg::Vec3 position = bs._center+osg::Vec3( 0.0,-3.5f * bs._radius,0.0f);
		
		osg::Vec3 up = osg::Vec3(0.0f,0.0f,1.0f);

		float centerDistance = (position-bs.center()).length();

		float znear = centerDistance-bs.radius();
		float zfar  = centerDistance+bs.radius();
		float zNearRatio = 0.001f;
		if (znear<zfar*zNearRatio) znear = zfar*zNearRatio;

#if 0
		// hack to illustrate the precision problems of excessive gap between near far range.
		znear = 0.00001*zfar;
#endif
		float top   = (bs.radius()/centerDistance)*znear;
		float right = top;

		//viewer.getCamera()->setReferenceFrame(osg::Camera::ABSOLUTE_RF);
		camera->setProjectionMatrixAsFrustum(-right,right,-top,top,znear,zfar);
		camera->setViewMatrixAsLookAt(position,bs.center(),up);

		//camera->setComputeNearFarMode(osg::CullSettings::DO_NOT_COMPUTE_NEAR_FAR);
	}
}

// Callback to be attached to camera rendering haptics view. Will start haptic rendering frame

void HapticDevicePreRenderCallback::operator()( const osg::Camera & camera) const
{
	const osg::GraphicsContext::Traits* traits = camera.getGraphicsContext()->getTraits();

	osg::Matrixd modelView, view, projection;
	const osg::Viewport *viewp = camera.getViewport();
	GLint viewport[4] = { viewp->x(), viewp->y(), viewp->width(), viewp->height() };

	view = camera.getViewMatrix();
	projection = camera.getProjectionMatrix();
	osg::MatrixList& matrixList = camera.getWorldMatrices();

	osg::MatrixList::iterator it = matrixList.begin();

	for(; it != matrixList.end(); it++)
	{
		modelView.postMult(*it);
	}

	modelView = modelView*view;


	//--by SophiaSoo/CUHK: for two arms
	m_device->makeCurrentDevice();

	GLint mode;
	glGetIntegerv(GL_MATRIX_MODE, &mode);
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadMatrixd(modelView.ptr());
	m_device->beginFrame();

	m_device->updateWorkspace( traits->width, traits->height,
		modelView, view, projection, viewport);
	glPopMatrix();
	glMatrixMode(mode);

}
