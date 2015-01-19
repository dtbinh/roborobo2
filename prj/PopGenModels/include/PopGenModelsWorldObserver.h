/**
 * @author Nicolas Bredeche <nicolas.bredeche@upmc.fr>
 * NNlib: Leo Cazenille <leo.cazenille@upmc.fr>
 */





#ifndef POPGENMODELSWORLDOBSERVER_H
#define POPGENMODELSWORLDOBSERVER_H

#include "RoboroboMain/common.h"
#include "RoboroboMain/roborobo.h"
#include "Observers/Observer.h"
#include "Observers/WorldObserver.h"
#include "WorldModels/RobotWorldModel.h"
#include "PopGenModels/include/PopGenModelsSharedData.h"
#include "PopGenModels/include/PopGenModelsSharedData.h"

//class World;

class PopGenModelsWorldObserver : public WorldObserver
{
private:
    void updateEnvironment();
    void updateMonitoring();
    void monitorPopulation( bool localVerbose = true );
    
protected:
    int _generationCount;
    int _generationItCount;
    
public:
    PopGenModelsWorldObserver(World *world);
    ~PopGenModelsWorldObserver();
    
    void reset();
    void step();
    
    int getGenerationItCount() { return _generationItCount; }

};

#endif
