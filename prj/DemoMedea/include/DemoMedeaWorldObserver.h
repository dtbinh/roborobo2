/**
 * @author Nicolas Bredeche <nicolas.bredeche@upmc.fr>
 * NNlib: Leo Cazenille <leo.cazenille@upmc.fr>
 */





#ifndef DEMOMEDEAWORLDOBSERVER_H
#define DEMOMEDEAWORLDOBSERVER_H

#include "RoboroboMain/common.h"
#include "RoboroboMain/roborobo.h"
#include "Observers/Observer.h"
#include "Observers/WorldObserver.h"
#include "WorldModels/RobotWorldModel.h"
#include "DemoMedea/include/DemoMedeaSharedData.h"
#include "DemoMedea/include/DemoMedeaSharedData.h"

//class World;

class DemoMedeaWorldObserver : public WorldObserver
{
private:
    void updateEnvironment();
    void updateMonitoring();
    
protected:
    int _generationCount;
    int _lifeIterationCount;
    
public:
    DemoMedeaWorldObserver(World *world);
    ~DemoMedeaWorldObserver();
    
    void reset();
    void step();
    
    int getLifeIterationCount() { return _lifeIterationCount; }

};

#endif
