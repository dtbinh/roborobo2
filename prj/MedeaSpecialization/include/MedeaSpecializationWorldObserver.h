/**
 * @author Nicolas Bredeche <nicolas.bredeche@upmc.fr>
 * NNlib: Leo Cazenille <leo.cazenille@upmc.fr>
 */





#ifndef MEDEASPECIALIZATIONWORLDOBSERVER_H
#define MEDEASPECIALIZATIONWORLDOBSERVER_H

#include "RoboroboMain/common.h"
#include "RoboroboMain/roborobo.h"
#include "Observers/Observer.h"
#include "Observers/WorldObserver.h"
#include "WorldModels/RobotWorldModel.h"
#include "MedeaSpecialization/include/MedeaSpecializationSharedData.h"
#include "MedeaSpecialization/include/MedeaSpecializationSharedData.h"

//class World;

class MedeaSpecializationWorldObserver : public WorldObserver
{
private:
    void updateEnvironment();
    void updateMonitoring();
    void monitorPopulation( bool localVerbose = true );
    
    std::vector<Point2d> positions;// = new std::vector<Point2d>(0);
    int posIndex_landmark1;
    int posIndex_landmark2;

protected:
    int _generationCount;
    int _generationItCount;
    
public:
    MedeaSpecializationWorldObserver(World *world);
    ~MedeaSpecializationWorldObserver();
    
    void reset();
    void step();
    
    int getGenerationItCount() { return _generationItCount; }

};

#endif
