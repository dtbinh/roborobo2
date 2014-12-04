/**
 * @author Nicolas Bredeche <nicolas.bredeche@upmc.fr>
 * NNlib: Leo Cazenille <leo.cazenille@upmc.fr>
 */





#ifndef BOIDSWORLDOBSERVER_H
#define BOIDSWORLDOBSERVER_H

#include "RoboroboMain/common.h"
#include "RoboroboMain/roborobo.h"
#include "Observers/Observer.h"
#include "Observers/WorldObserver.h"
#include "WorldModels/RobotWorldModel.h"
#include "Boids/include/BoidsSharedData.h"
#include "Boids/include/BoidsSharedData.h"

//class World;

class BoidsWorldObserver : public WorldObserver
{
private:
    void updateEnvironment();
    void updateMonitoring();
    void monitorPopulation( bool localVerbose = true );
    
protected:
    int _generationCount;
    int _generationItCount;
    
public:
    BoidsWorldObserver(World *world);
    ~BoidsWorldObserver();
    
    void reset();
    void step();
    
    int getGenerationItCount() { return _generationItCount; }

};

#endif
