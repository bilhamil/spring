/* This file is part of the Spring engine (GPL v2 or later), see LICENSE.html */

#ifndef TUIOHANDLER_H
#define TUIOHANDLER_H

#include <string>
#include <vector>
#include <map>
#include <hash_map>
#include <hash_set>
#include "lib/tuio/TuioClient.h"
#include "lib/tuio/TuioObject.h"

class CInputReceiver;
class CCameraController;

struct CInputReceiver_hash
{
    size_t operator()(const CInputReceiver *p) const
    {
        return reinterpret_cast<size_t>(p) ;
    }
};

class CTuioHandler : public TUIO::TuioListener
{

public:
	CTuioHandler(int port = 3333);
	~CTuioHandler();

    void addTuioObject(TUIO::TuioObject *tobj);
    void updateTuioObject(TUIO::TuioObject *tobj);
    void removeTuioObject(TUIO::TuioObject *tobj);
    void addTuioCursor(TUIO::TuioCursor *tcur);
    void updateTuioCursor(TUIO::TuioCursor *tcur);
    void removeTuioCursor(TUIO::TuioCursor *tcur);
    void refresh(TUIO::TuioTime ftime);

    void lock();
    void unlock();

private:
	TUIO::TuioClient* client;
	__gnu_cxx::hash_map<int, CInputReceiver*> activeReceivers;
	__gnu_cxx::hash_set<CInputReceiver*, CInputReceiver_hash> refreshedReceivers;
	bool cameraReceiving;
};

extern CTuioHandler* tuio;

#endif /* TUIOHANDLER_H */
