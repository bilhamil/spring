/* This file is part of the Spring engine (GPL v2 or later), see LICENSE.html */

#include "StdAfx.h"
#include <SDL_keysym.h>

#include "mmgr.h"

#include "FreeController.h"

#include "ConfigHandler.h"
#include "Game/Camera.h"
#include "LogOutput.h"
#include "Map/Ground.h"
#include "GlobalUnsynced.h"
#include "Rendering/GlobalRendering.h"
#include <boost/cstdint.hpp>
#include "System/Matrix44f.h"
#include "Game/UI/TUIOHandler.h"


using std::max;
using std::min;
extern boost::uint8_t *keys;

/******************************************************************************/
/******************************************************************************/
//
//  TODO: - separate speed variable for tracking state
//        - smooth ransitions between tracking states and units
//        - improve controls
//        - rename it?  ;-)
//

CFreeController::CFreeController()
: vel(0.0f, 0.0f, 0.0f),
  avel(0.0f, 0.0f, 0.0f),
  prevVel(0.0f, 0.0f, 0.0f),
  prevAvel(0.0f, 0.0f, 0.0f),
  tracking(false),
  trackPos(0.0f, 0.0f, 0.0f),
  trackRadius(0.0f),
  gndLock(false),
  cursors(),
  lastNumCursors(0),
  cursorNumberFrameCount(0)
{
	dir = float3(0.0f, -2.0f, -1.0f);
	dir.ANormalize();
	if (camera) {
		const float hDist = sqrt((dir.x * dir.x) + (dir.z * dir.z));
		camera->rot.y = atan2(dir.x, dir.z);
		camera->rot.x = atan2(dir.y, hDist);
	}
	pos -= (dir * 1000.0f);

	enabled     = !!configHandler->Get("CamFreeEnabled",   0);
	invertAlt   = !!configHandler->Get("CamFreeInvertAlt", 0);
	goForward   = !!configHandler->Get("CamFreeGoForward", 0);
	fov         = configHandler->Get("CamFreeFOV",           45.0f);
	scrollSpeed = configHandler->Get("CamFreeScrollSpeed",  500.0f);
	gravity     = configHandler->Get("CamFreeGravity",     -500.0f);
	slide       = configHandler->Get("CamFreeSlide",          0.5f);
	gndOffset   = configHandler->Get("CamFreeGroundOffset",  16.0f);
	tiltSpeed   = configHandler->Get("CamFreeTiltSpeed",    150.0f);
	tiltSpeed   = tiltSpeed * (PI / 180.0);
	autoTilt    = configHandler->Get("CamFreeAutoTilt",     150.0f);
	autoTilt    = autoTilt * (PI / 180.0);
	velTime     = configHandler->Get("CamFreeVelTime",        1.5f);
	velTime     = max(0.1f, velTime);
	avelTime    = configHandler->Get("CamFreeAngVelTime",     1.0f);
	avelTime    = max(0.1f, avelTime);

	lastSinglePoint.x = -1;
	lastSinglePoint.y = -1;
}


void CFreeController::SetTrackingInfo(const float3& target, float radius)
{
	tracking = true;
	trackPos = target;
	trackRadius = radius;;

	// lock the view direction to the target
	const float3 diff(trackPos - pos);
	const float rads = atan2(diff.x, diff.z);
	camera->rot.y = rads;

	const float len2D = diff.Length2D();
	if (fabs(len2D) <= 0.001f) {
		camera->rot.x = 0.0f;
	} else {
		camera->rot.x = atan2((trackPos.y - pos.y), len2D);
	}

	camera->UpdateForward();
}


void CFreeController::Update()
{
	if (!globalRendering->active) {
		vel  = ZeroVector;
		avel = ZeroVector;
		prevVel  = vel;
		prevAvel = avel;
		return;
	}

	// safeties
	velTime  = max(0.1f,  velTime);
	avelTime = max(0.1f, avelTime);

	// save some old state
	const float ctrlVelY = vel.y;
	const float3 prevPos = pos;

	// setup the time fractions
	const float ft = globalRendering->lastFrameTime;
	const float nt = (ft / velTime); // next time factor
	const float pt = (1.0f - nt);    // prev time factor
	const float ant = (ft / avelTime); // next time factor
	const float apt = (1.0f - ant);    // prev time factor

	// adjustment to match the ground slope
	float autoTiltVel = 0.0f;
	if (gndLock && (autoTilt > 0.0f)) {
		const float gndHeight = ground->GetHeight2(pos.x, pos.z);
		if (pos.y < (gndHeight + gndOffset + 1.0f)) {
			float3 hDir;
			hDir.y = 0.0f;
			hDir.x = (float)sin(camera->rot.y);
			hDir.z = (float)cos(camera->rot.y);
			const float3 gndNormal = ground->GetSmoothNormal(pos.x, pos.z);
			const float dot = gndNormal.dot(hDir);
			const float gndRotX = (float)acos(dot) - (PI * 0.5f);
			const float rotXdiff = (gndRotX - camera->rot.x);
			autoTiltVel = (autoTilt * rotXdiff);
		}
	}

	// convert control velocity into position velocity
	/*if (!tracking) {
		if (goForward) {
			const float3 tmpVel((camera->forward * vel.x) +
			                    (UpVector        * vel.y) +
			                    (camera->right   * vel.z));
			vel = tmpVel;
		}
		else {
			float3 forwardNoY(camera->forward.x, 0.0f, camera->forward.z);
			forwardNoY.ANormalize();
			const float3 tmpVel((forwardNoY    * vel.x) +
			                    (UpVector      * vel.y) +
			                    (camera->right * vel.z));
			vel = tmpVel;
		}
	}*/

    float3 forwardNoY = UpVector.cross(camera->right);//(camera->forward.x, 0.0f, camera->forward.z);
    forwardNoY.ANormalize();
    const float3 tmpVel((forwardNoY    * vel.x) +
                        (UpVector      * vel.y) +
                        (camera->right * vel.z));

    vel = tmpVel;

	// smooth the velocities
	vel  =  (vel * nt)  +  (prevVel * pt);
	avel = (avel * ant) + (prevAvel * apt);

	// no smoothing for gravity (still isn't right)
	if (gndLock) {
		const float dGrav = (gravity * ft);
		vel.y += dGrav;
		if (slide > 0.0f) {
			const float gndHeight = ground->GetHeight2(pos.x, pos.z);
			if (pos.y < (gndHeight + gndOffset + 1.0f)) {
				const float3 gndNormal = ground->GetSmoothNormal(pos.x, pos.z);
				const float dotVal = gndNormal.y;
				const float scale = (dotVal * slide * -dGrav);
				vel.x += (gndNormal.x * scale);
				vel.z += (gndNormal.z * scale);
			}
		}
	}

	// set the new position/rotation
	if (!tracking) {
		pos           += (vel         * ft);
		camera->rot   += (avel        * ft);
		camera->rot.x += (autoTiltVel * ft); // note that this is not smoothed
	}
	else {
		// speed along the tracking direction varies with distance
		const float3 diff = (pos - trackPos);
		if (goForward) {
			const float dist = max(0.1f, diff.Length());
			const float nomDist = 512.0f;
			float speedScale = (dist / nomDist);
			speedScale = max(0.25f, min(16.0f, speedScale));
			const float delta = -vel.x * (ft * speedScale);
			const float newDist = max(trackRadius, (dist + delta));
			const float scale = (newDist / dist);
			pos = trackPos + (diff * scale);
			pos.y += (vel.y * ft);
		}
		else {
			const float dist = max(0.1f, diff.Length2D());
			const float nomDist = 512.0f;
			float speedScale = (dist / nomDist);
			speedScale = max(0.25f, min(16.0f, speedScale));
			const float delta = -vel.x * (ft * speedScale);
			const float newDist = max(trackRadius, (dist + delta));
			const float scale = (newDist / dist);
			pos.x = trackPos.x + (scale * diff.x);
			pos.z = trackPos.z + (scale * diff.z);
			pos.y += (vel.y * ft);
		}

		// convert the angular velocity into its positional change
		const float3 diff2 = (pos - trackPos);
		const float deltaRad = (avel.y * ft);
		const float cos_val = cos(deltaRad);
		const float sin_val = sin(deltaRad);
		pos.x = trackPos.x + ((cos_val * diff2.x) + (sin_val * diff2.z));
		pos.z = trackPos.z + ((cos_val * diff2.z) - (sin_val * diff2.x));
	}

	// setup ground lock
	const float gndHeight = ground->GetHeight2(pos.x, pos.z);
	if (keys[SDLK_LSHIFT]) {
		if (ctrlVelY > 0.0f) {
			gndLock = false;
		} else if ((gndOffset > 0.0f) && (ctrlVelY < 0.0f) &&
		           (pos.y < (gndHeight + gndOffset))) {
			gndLock = true;
		}
	}

	// positional clamps
	if (gndOffset < 0.0f) {
		pos.y = (gndHeight - gndOffset);
		vel.y = 0.0f;
	}
	else if (gndLock && (gravity >= 0.0f)) {
		pos.y = (gndHeight + gndOffset);
		vel.y = 0.0f;
	}
	else if (gndOffset > 0.0f) {
		const float minHeight = (gndHeight + gndOffset);
		if (pos.y < minHeight) {
			pos.y = minHeight;
			if (gndLock) {
				vel.y = min(fabs(scrollSpeed), ((minHeight - prevPos.y) / ft));
			} else {
				vel.y = 0.0f;
			}
		}
	}

	// angular clamps
	const float xRotLimit = (PI * 0.4999f);
	if (camera->rot.x > xRotLimit) {
		camera->rot.x = xRotLimit;
		avel.x = 0.0f;
	}
	else if (camera->rot.x < -xRotLimit) {
		camera->rot.x = -xRotLimit;
		avel.x = 0.0f;
	}
	camera->rot.y = fmod(camera->rot.y, PI * 2.0f);

	// setup for the next loop
	prevVel  = vel;
	prevAvel = avel;
	vel  = ZeroVector;
	avel = ZeroVector;

	tracking = false;
}


void CFreeController::KeyMove(float3 move)
{
	const float qy = (move.y == 0.0f) ? 0.0f : (move.y > 0.0f ? 1.0f : -1.0f);
	const float qx = (move.x == 0.0f) ? 0.0f : (move.x > 0.0f ? 1.0f : -1.0f);

	const float speed  = keys[SDLK_LMETA] ? 4.0f * scrollSpeed : scrollSpeed;
	const float aspeed = keys[SDLK_LMETA] ? 2.0f * tiltSpeed   : tiltSpeed;

	if (keys[SDLK_LCTRL]) {
		avel.x += (aspeed * -qy); // tilt
	}
	else if (keys[SDLK_LSHIFT]) {
		vel.y += (speed * -qy); // up/down
	}
	else {
		vel.x += (speed * qy); // forwards/backwards
	}

	if (tracking) {
		avel.y += (aspeed * qx); // turntable rotation
	}
	else if (!keys[SDLK_LALT] == invertAlt) {
		vel.z += (speed * qx); // left/right
	}
	else {
		avel.y += (aspeed * -qx); // spin
	}

	return;
}


void CFreeController::MouseMove(float3 move)
{
	boost::uint8_t prevAlt   = keys[SDLK_LALT];
	boost::uint8_t prevCtrl  = keys[SDLK_LCTRL];
	boost::uint8_t prevShift = keys[SDLK_LSHIFT];

	keys[SDLK_LCTRL] = !keys[SDLK_LCTRL]; // tilt
	keys[SDLK_LALT] = (invertAlt == !keys[SDLK_LALT]);
	KeyMove(move);

	keys[SDLK_LALT] = prevAlt;
	keys[SDLK_LCTRL] = prevCtrl;
	keys[SDLK_LSHIFT] = prevShift;
}


void CFreeController::ScreenEdgeMove(float3 move)
{
	boost::uint8_t prevAlt   = keys[SDLK_LALT];
	boost::uint8_t prevCtrl  = keys[SDLK_LCTRL];
	boost::uint8_t prevShift = keys[SDLK_LSHIFT];

	keys[SDLK_LALT] = (invertAlt == !keys[SDLK_LALT]);
	KeyMove(move);

	keys[SDLK_LALT] = prevAlt;
	keys[SDLK_LCTRL] = prevCtrl;
	keys[SDLK_LSHIFT] = prevShift;
}

void CFreeController::MouseWheelMove(float move)
{
	/*boost::uint8_t prevCtrl  = keys[SDLK_LCTRL];
	boost::uint8_t prevShift = keys[SDLK_LSHIFT];
	keys[SDLK_LCTRL] = 0;
	keys[SDLK_LSHIFT] = 1;
	const float3 m(0.0f, move, 0.0f);
	KeyMove(m);
	keys[SDLK_LCTRL] = prevCtrl;
	keys[SDLK_LSHIFT] = prevShift;*/
}

 /* tuio updates */
void CFreeController::addTuioCursor(TUIO::TuioCursor *tcur)
{
    cursors[tcur->getCursorID()] = tcur;
    vel = ZeroVector;
    prevVel = ZeroVector;
}

void CFreeController::updateTuioCursor(TUIO::TuioCursor *tcur)
{

}

void CFreeController::removeTuioCursor(TUIO::TuioCursor *tcur)
{
    int numCursors = cursors.size();
    if(cursors.find(tcur->getCursorID()) != cursors.end())
    {
        cursors.erase(cursors.find(tcur->getCursorID()));
    }
    if(numCursors == 1)
    {
        float3 forwardNoY = UpVector.cross(camera->right);
        vel.x = lastVel.dot(forwardNoY);
        vel.z = lastVel.dot(camera->right);
        vel.y = lastVel.dot(UpVector);
    }
}

float3 CFreeController::translate(shortint2 &last, shortint2 &now)
{
    shortint2 gp = now;
    clampToWindowSpace(now);

    if(isInWindowSpace(gp))
    {

        float3 newPos = pos;

        float3 oldDir = camera->CalcPixelDir(last.x,last.y).SafeNormalize();
        float3 newDir = camera->CalcPixelDir(now.x,now.y).SafeNormalize();

        float oldDist= -pos.y / oldDir.y;//ground->LineGroundCol(pos,pos+oldDir*(globalRendering->viewRange*1.4f));

        //float newDist=ground->LineGroundCol(pos,pos+newDir*(globalRendering->viewRange*1.4f));

        if(oldDist > 0)
        {
            float3 oldGroundPos = oldDir * oldDist;

            float newDist = (-pos.y) / newDir.y;

            float3 newGroundPos = newDir * newDist;

            float3 dpos = newGroundPos - oldGroundPos;

            dpos.y = 0;

            dpos = -dpos;

            pos += dpos;

            return dpos;
        }

    }
    return ZeroVector;
}

void CFreeController::tuioRefresh(TUIO::TuioTime ftime)
{
    int numCursors = cursors.size();

    if(lastNumCursors != numCursors)
    {
        cursorNumberFrameCount = 0;
    }

    if(numCursors != 1)
    {
        //vel.x = 0;
        //vel.z = 0;
        lastSinglePoint.x = lastSinglePoint.y = -1;
        lastVel = ZeroVector;
    }
    if(numCursors != 2)
    {
        prevDist = -1;
        lastMidPoint.x = lastMidPoint.y = -1;
    }
    if(numCursors != 3)
    {
        lastThreeFingerDx = -1;
    }

    switch(numCursors)
    {
        case 1:
        {
            TUIO::TuioCursor *tcur = cursors.begin()->second;

            shortint2 np = toWindowSpace(tcur);

            if(lastSinglePoint.x > 0)
            {
                float seconds = ((ftime - lastTime).getTotalMilliseconds()) / 1000.0f;
                float3 disp = translate(lastSinglePoint, np);
                if(seconds > 0.001f)
                {
                    if(lastVel.x == 0 && lastVel.y == 0 && lastVel.z == 0)
                    {
                        lastVel = disp / (seconds / 100);
                    }
                    else
                    {
                        lastVel = lastVel * 0.9f + disp * 0.1f / (seconds / 100);
                    }
                }

            }

            //always change the lastSinglePort
            lastSinglePoint = np;
        }
        break;

        case 2:
        {
            TUIO::TuioCursor* one = cursors.begin()->second;
            TUIO::TuioCursor* two = (++cursors.begin())->second;

            shortint2 oneScr, twoScr;
            oneScr = toWindowSpace(one);
            twoScr = toWindowSpace(two);

            shortint2 mid;
            mid.x = (oneScr.x + twoScr.x) / 2;
            mid.y = (oneScr.y + twoScr.y) / 2;


            /* do any panning that we're going to do */
            {
                shortint2 midCopy = mid;

                if(lastMidPoint.x > 0)
                {
                    translate(lastMidPoint, midCopy);
                }

                //always change the lastSinglePort
                lastMidPoint = midCopy;
            }

            int xdif = (oneScr.x - twoScr.x);

            int ydif = (oneScr.y - twoScr.y);

            float dist = ::sqrtf(xdif * xdif + ydif * ydif);
            float curRot = -one->getAngle(two);

            //logOutput.Print("two %f", dist);
            if(prevDist >= 0)
            {

                shortint2 mid;
                mid.x = (oneScr.x + twoScr.x) / 2;
                mid.y = (oneScr.y + twoScr.y) / 2;

                if(isInWindowSpace(mid))
                {
                    float dDist = dist - prevDist;
                    int changeUnit = 5;

                    float3 midDir = camera->CalcPixelDir(mid.x,mid.y).SafeNormalize();
                    float grnDist=ground->LineGroundCol(pos,pos+midDir*(globalRendering->viewRange*1.4f));

                    if(streflop::fabsf(dDist) >= changeUnit)
                    {

                        float3 midLoc = pos + midDir * grnDist;

                        float3 oneDir = camera->CalcPixelDir(oneScr.x, oneScr.y).SafeNormalize();

                        shortint2 oldOnePos = mid;
                        oldOnePos.x += (oneScr.x - mid.x) * (prevDist / dist);
                        oldOnePos.y += (oneScr.y - mid.y) * (prevDist / dist);

                        float3 oldOneDir = camera->CalcPixelDir(oldOnePos.x, oldOnePos.y).SafeNormalize();

                        float oneDist = grnDist / (oneDir.dot(midDir));
                        float oldOneDist = grnDist / (oldOneDir.dot(midDir));

                        float3 oneLoc = pos + oneDir * oneDist;
                        float3 oldOneLoc = pos + oldOneDir * oldOneDist;

                        float newD = (midLoc - oneLoc).Length();
                        float oldD = (midLoc - oldOneLoc).Length();

                        float newGrnDist = oldD * grnDist / newD;

                        pos += midDir * (grnDist - newGrnDist);

                        /* update prevDist */
                        prevDist = dist;
                    }

                    float dRot = curRot - prevRot;
                    dRot = streflop::fmodf(dRot, M_PI * 2);
                    float angularChangeThresh = 1.0f * M_PI / 180.0f;

                    if(streflop::fabsf(dRot) >= angularChangeThresh && grnDist >= 0)
                    {
                        CMatrix44f rotate;
                        rotate.Rotate(dRot, UpVector);

                        float3 grndOffset = midDir * -grnDist;

                        float3 newOffset = rotate.Mul(grndOffset);

                        newOffset -= grndOffset;

                        pos += newOffset;

                        camera->rot.y += dRot;

                        camera->UpdateForward();

                        prevRot = curRot;
                    }

                }

            }
            else
            {
                prevDist = dist;
                prevRot = curRot;
            }
        }
        break;

        case 3:
        {
            shortint2 highest, lowest;

            lowest.y = -1;
            highest.y = 10000;

            for(std::map<int, TUIO::TuioCursor*>::const_iterator it = cursors.begin(); it != cursors.end(); it++)
            {
                TUIO::TuioCursor* cursor = it->second;
                shortint2 scrCur = toWindowSpace(cursor);

                if(scrCur.y <= highest.y)
                {
                    highest = scrCur;
                }

                if(scrCur.y >= lowest.y)
                {
                    lowest = scrCur;
                }

            }

            int dX = lowest.y - highest.y;

            if(lastThreeFingerDx >= 0)
            {
                int dDx = dX - lastThreeFingerDx;

                int thresh = 1;

                if(abs(dDx) >= thresh)
                {
                    int usedChange = dDx / thresh;
                    usedChange *= thresh;

                    lastThreeFingerDx += usedChange;

                    float rot = -1.0f / 180.0f * M_PI * usedChange / 5.0f;



                    float3 rotArm = camera->CalcPixelDir(highest.x,highest.y).SafeNormalize();

                    float currentRot = camera->rot.x;

                    // angular clamps
                    const float xRotLimit = - 30.0f * PI / 180.0f;
                    const float lowerRotLimit = -(PI * 0.4999f);
                    if (currentRot + rot > xRotLimit) {
                        rot = xRotLimit - currentRot;
                    }
                    else if (currentRot + rot < lowerRotLimit) {
                        rot = lowerRotLimit - currentRot;
                    }

                    CMatrix44f rotate;
                    rotate.Rotate(rot, camera->right);

                    float grnDist=ground->LineGroundCol(pos,pos+rotArm*(globalRendering->viewRange*1.4f));

                    if(grnDist > 0)
                    {
                        rotArm *= -grnDist;

                        float3 rotatedArm = rotate.Mul(rotArm);

                        pos += (rotatedArm - rotArm);

                        camera->rot.x +=  rot;
                    }
                    //camera->UpdateForward();*/

                    //camera->Pitch(rot);

                }
            }
            else
            {
                lastThreeFingerDx = dX;
            }
        }
        break;
        default:
        break;
    };

    lastTime = ftime;
    lastNumCursors = numCursors;
    cursorNumberFrameCount++;
}


void CFreeController::SetPos(const float3& newPos)
{
	const float h = ground->GetHeight2(newPos.x, newPos.z);
	const float3 target = float3(newPos.x, h, newPos.z);
//	const float3 target = newPos;
	const float yDiff = pos.y - target.y;
	if ((yDiff * dir.y) >= 0.0f) {
		pos = float3(newPos.x, h, newPos.z);
	} else {
		pos = target - (dir * fabs(yDiff / dir.y));
	} // FIXME
/*
	const float oldPosY = pos.y;
	CCameraController::SetPos(newPos);
	pos.y = oldPosY;
	if (gndOffset != 0.0f) {
		const float h = ground->GetHeight2(pos.x, pos.z);
		const float absH = h + fabsf(gndOffset);
		if (pos.y < absH) {
			pos.y = absH;
		}
	}
*/
	prevVel  = ZeroVector;
	prevAvel = ZeroVector;
}


float3 CFreeController::GetPos()
{
	return pos;
}


float3 CFreeController::GetDir()
{
	dir.x = (float)(sin(camera->rot.y) * cos(camera->rot.x));
	dir.z = (float)(cos(camera->rot.y) * cos(camera->rot.x));
	dir.y = (float)(sin(camera->rot.x));
	dir.ANormalize();
	return dir;
}


float3 CFreeController::SwitchFrom() const
{
	const float x = max(0.1f, min(float3::maxxpos - 0.1f, pos.x));
	const float z = max(0.1f, min(float3::maxzpos - 0.1f, pos.z));
	return float3(x, ground->GetHeight(x, z) + 5.0f, z);
}


void CFreeController::SwitchTo(bool showText)
{
	if (showText) {
		logOutput.Print("Switching to Free style camera");
	}
	prevVel  = ZeroVector;
	prevAvel = ZeroVector;
	cursors.clear();
}


void CFreeController::GetState(StateMap& sm) const
{
	sm["px"] = pos.x;
	sm["py"] = pos.y;
	sm["pz"] = pos.z;

	sm["dx"] = dir.x;
	sm["dy"] = dir.y;
	sm["dz"] = dir.z;

	sm["rx"] = camera->rot.x;
	sm["ry"] = camera->rot.y;
	sm["rz"] = camera->rot.z;

	sm["fov"]         = fov;
	sm["gndOffset"]   = gndOffset;
	sm["gravity"]     = gravity;
	sm["slide"]       = slide;
	sm["scrollSpeed"] = scrollSpeed;
	sm["tiltSpeed"]   = tiltSpeed;
	sm["velTime"]     = velTime;
	sm["avelTime"]    = avelTime;
	sm["autoTilt"]    = autoTilt;

	sm["goForward"]   = goForward ? +1.0f : -1.0f;
	sm["invertAlt"]   = invertAlt ? +1.0f : -1.0f;
	sm["gndLock"]     = gndLock   ? +1.0f : -1.0f;

	sm["vx"] = prevVel.x;
	sm["vy"] = prevVel.y;
	sm["vz"] = prevVel.z;

	sm["avx"] = prevAvel.x;
	sm["avy"] = prevAvel.y;
	sm["avz"] = prevAvel.z;
}


bool CFreeController::SetState(const StateMap& sm)
{
	SetStateFloat(sm, "px", pos.x);
	SetStateFloat(sm, "py", pos.y);
	SetStateFloat(sm, "pz", pos.z);

	SetStateFloat(sm, "dx", dir.x);
	SetStateFloat(sm, "dy", dir.y);
	SetStateFloat(sm, "dz", dir.z);

	SetStateFloat(sm, "rx", camera->rot.x);
	SetStateFloat(sm, "ry", camera->rot.y);
	SetStateFloat(sm, "rz", camera->rot.z);

	SetStateFloat(sm, "fov",         fov);
	SetStateFloat(sm, "gndOffset",   gndOffset);
	SetStateFloat(sm, "gravity",     gravity);
	SetStateFloat(sm, "slide",       slide);
	SetStateFloat(sm, "scrollSpeed", scrollSpeed);
	SetStateFloat(sm, "tiltSpeed",   tiltSpeed);
	SetStateFloat(sm, "velTime",     velTime);
	SetStateFloat(sm, "avelTime",    avelTime);
	SetStateFloat(sm, "autoTilt",    autoTilt);

	SetStateBool (sm, "goForward",   goForward);
	SetStateBool (sm, "invertAlt",   invertAlt);
	SetStateBool (sm, "gndLock",     gndLock);

	SetStateFloat(sm, "vx", prevVel.x);
	SetStateFloat(sm, "vy", prevVel.y);
	SetStateFloat(sm, "vz", prevVel.z);

	SetStateFloat(sm, "avx", prevAvel.x);
	SetStateFloat(sm, "avy", prevAvel.y);
	SetStateFloat(sm, "avz", prevAvel.z);

	return true;
}
