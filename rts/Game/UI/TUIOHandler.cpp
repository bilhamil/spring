/* This file is part of the Spring engine (GPL v2 or later), see LICENSE.html */

#include "StdAfx.h"

#include "mmgr.h"

#include <algorithm>
#include <boost/cstdint.hpp>

#include "TUIOHandler.h"
#include "Game/CameraHandler.h"
#include "Game/Camera/CameraController.h"
#include "Game/Camera.h"
#include "System/LogOutput.h"
#include "CommandColors.h"
#include "InputReceiver.h"
#include "GuiHandler.h"
#include "MiniMap.h"
#include "MouseCursor.h"
#include "System/Input/MouseInput.h"
#include "TooltipConsole.h"
#include "Sim/Units/Groups/Group.h"
#include "Game/Game.h"
#include "Game/GameHelper.h"
#include "Game/SelectedUnits.h"
#include "Game/PlayerHandler.h"
#include "Game/UI/UnitTracker.h"
#include "Map/Ground.h"
#include "Map/MapDamage.h"
#include "Lua/LuaInputReceiver.h"
#include "ConfigHandler.h"
#include "Rendering/GlobalRendering.h"
#include "Rendering/glFont.h"
#include "Rendering/GL/myGL.h"
#include "Rendering/InMapDraw.h"
#include "Rendering/Textures/Bitmap.h"
#include "Sim/Features/FeatureDef.h"
#include "Sim/Features/Feature.h"
#include "Sim/Misc/LosHandler.h"
#include "Sim/Misc/TeamHandler.h"
#include "Sim/Units/UnitDef.h"
#include "Sim/Units/Unit.h"
#include "Sim/Units/UnitHandler.h"
#include "EventHandler.h"
#include "Exceptions.h"
#include "FastMath.h"
#include "myMath.h"
#include "Sound/ISound.h"
#include "Sound/IEffectChannel.h"

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

extern boost::uint8_t *keys;


CTuioHandler* tuio = NULL;

CTuioHandler::CTuioHandler(int port): activeReceivers(), refreshedReceivers(), cameraReceiving(false)
{
    client = new SDLTuioClient(port);
    client->addTuioListener(this);
    client->connect();
}

CTuioHandler::~CTuioHandler()
{
    client->disconnect();
    delete client;
}

 void CTuioHandler::addTuioObject(TUIO::TuioObject *tobj)
{
    logOutput.Print("Add Object:");
}

void CTuioHandler::updateTuioObject(TUIO::TuioObject *tobj)
{
    logOutput.Print("Update Object:");
}

void CTuioHandler::removeTuioObject(TUIO::TuioObject *tobj)
{
    logOutput.Print("Remove Object:");
}

void CTuioHandler::addTuioCursor(TUIO::TuioCursor *tcur)
{
    //logOutput.Print("Add Curser:");

    //taken from MouseHandler ....

	/*

	camHandler->GetCurrentController().MousePress(x, y, button);

	dir = hide? camera->forward: camera->CalcPixelDir(x, y);

 	buttons[button].chorded = buttons[SDL_BUTTON_LEFT].pressed ||
 	                          buttons[SDL_BUTTON_RIGHT].pressed;
	buttons[button].pressed = true;
	buttons[button].time = gu->gameTime;
	buttons[button].x = x;
	buttons[button].y = y;
	buttons[button].camPos = camera->pos;
	buttons[button].dir = dir;
	buttons[button].movement = 0;

	activeButton = button;


	if (activeReceiver && activeReceiver->MousePress(x, y, button))
		return;

	if(inMapDrawer &&  inMapDrawer->keyPressed){
		inMapDrawer->MousePress(x, y, button);
		return;
	}
    */

	std::deque<CInputReceiver*>& inputReceivers = GetInputReceivers();
	std::deque<CInputReceiver*>::iterator ri;

    //logOutput.Print("Add Curser: %d", tcur->getCursorID());

	if (!game->hideInterface) {
		for (ri = inputReceivers.begin(); ri != inputReceivers.end(); ++ri) {
			CInputReceiver* recv=*ri;
			if (recv && recv->addTuioCursor(tcur))
			{
				if (!activeReceivers[tcur->getCursorID()])
					activeReceivers[tcur->getCursorID()] = recv;
				return;
			}
		}
	} else {
		if (luaInputReceiver && luaInputReceiver->addTuioCursor(tcur)) {
            if (!activeReceivers[tcur->getCursorID()])
					activeReceivers[tcur->getCursorID()] = luaInputReceiver;
            return;
		}
		if (guihandler && guihandler->addTuioCursor(tcur)) {
			if (!activeReceivers[tcur->getCursorID()])
					activeReceivers[tcur->getCursorID()] = guihandler;
            return;
		}
	}

	cameraReceiving = true;
    camHandler->GetCurrentController().addTuioCursor(tcur);
}

void CTuioHandler::updateTuioCursor(TUIO::TuioCursor *tcur)
{
    //logOutput.Print("Update Curser:");

    CInputReceiver* recv = activeReceivers[tcur->getCursorID()];
    if(recv)
    {
        recv->updateTuioCursor(tcur);
    }
    else
    {
        cameraReceiving = true;
        camHandler->GetCurrentController().updateTuioCursor(tcur);
    }

}

void CTuioHandler::removeTuioCursor(TUIO::TuioCursor *tcur)
{
    //logOutput.Print("Remove Curser:");

    CInputReceiver* recv = activeReceivers[tcur->getCursorID()];
    if(recv)
    {
        recv->removeTuioCursor(tcur);
        activeReceivers.erase(tcur->getCursorID());
    }
    else
    {
        cameraReceiving = true;
        camHandler->GetCurrentController().removeTuioCursor(tcur);
    }
}

void CTuioHandler::refresh(TUIO::TuioTime ftime)
{
    //logOutput.Print("refresh:");
    refreshedReceivers.clear();
    __gnu_cxx::hash_map<int, CInputReceiver*>::const_iterator it;


    for(it = activeReceivers.begin(); it != activeReceivers.end(); it++)
    {
        CInputReceiver* recv = it->second;

        if(recv && refreshedReceivers.find(recv) == refreshedReceivers.end())
        {
            recv->tuioRefresh(ftime);
            refreshedReceivers.insert(recv);
        }
    }

    if(cameraReceiving)
        camHandler->GetCurrentController().tuioRefresh(ftime);

    cameraReceiving = false;
}

void CTuioHandler::lock()
{
    client->lockCursorList();
    client->lockObjectList();
}

void CTuioHandler::unlock()
{
    client->unlockCursorList();
    client->unlockObjectList();
}

static inline int min(int val1, int val2) {
	return val1 < val2 ? val1 : val2;
}
static inline int max(int val1, int val2) {
	return val1 > val2 ? val1 : val2;
}

shortint2 toWindowSpace(TUIO::TuioPoint *point)
{
    int nx = point->getScreenX(globalRendering->screenSizeX);
    int ny = point->getScreenY(globalRendering->screenSizeY);

    nx -= globalRendering->winPosX;
    ny -= (globalRendering->screenSizeY - globalRendering->winPosY - globalRendering->winSizeY);

    shortint2 pnt;
    pnt.x = nx;
    pnt.y = ny;

    return pnt;
}

void clampToWindowSpace(shortint2 &pnt)
{
    pnt.x = max(0, (int) pnt.x);
    pnt.x = min(globalRendering->viewSizeX, (int) pnt.x);

    pnt.y = max(0, (int) pnt.y);
    pnt.y = min(globalRendering->viewSizeY, (int )pnt.y);
}

bool isInWindowSpace(const shortint2 &pnt)
{
    return pnt.x > 0 && pnt.y > 0 && pnt.x < globalRendering->viewSizeX && pnt.y < globalRendering->viewSizeY;
}
