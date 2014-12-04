/**
 * @author Nicolas Bredeche <nicolas.bredeche@upmc.fr>
 * NNlib: Leo Cazenille <leo.cazenille@upmc.fr>
 */

#include "Observers/AgentObserver.h"
#include "Observers/WorldObserver.h"
#include "Boids/include/BoidsWorldObserver.h"
#include "Boids/include/BoidsController.h"
#include "World/World.h"


BoidsWorldObserver::BoidsWorldObserver( World* world ) : WorldObserver( world )
{
    _world = world;
    
    // ==== loading project-specific properties

    gProperties.checkAndGetPropertyValue("gEvaluationTime",&BoidsSharedData::gEvaluationTime,true);

    gProperties.checkAndGetPropertyValue("gMonitorPositions",&BoidsSharedData::gMonitorPositions,true);
    
    gProperties.checkAndGetPropertyValue("gSnapshots",&BoidsSharedData::gSnapshots,false);
    gProperties.checkAndGetPropertyValue("gSnapshotsFrequency",&BoidsSharedData::gSnapshotsFrequency,false);

    
    // ====
    
    if ( !gRadioNetwork)
    {
        std::cout << "Error : gRadioNetwork must be true." << std::endl;
        exit(-1);
    }
    
    // * iteration and generation counters
    
    _generationItCount = -1;
    _generationCount = -1;
    
}

BoidsWorldObserver::~BoidsWorldObserver()
{
    // nothing to do.
}

void BoidsWorldObserver::reset()
{
    // nothing to do.
}

void BoidsWorldObserver::step()
{
    _generationItCount++;
    
    if( _generationItCount == BoidsSharedData::gEvaluationTime+1 ) // switch to next generation.
    {
        // update iterations and generations counters
        _generationItCount = 0;
        _generationCount++;
    }
    
    updateMonitoring();
    
    updateEnvironment();
    
}


void BoidsWorldObserver::updateEnvironment()
{
    // ...
}

void BoidsWorldObserver::updateMonitoring()
{
    // * Log at end of each generation

    if( gWorld->getIterations() % BoidsSharedData::gEvaluationTime == 1 || gWorld->getIterations() % BoidsSharedData::gEvaluationTime == BoidsSharedData::gEvaluationTime-1 ) // beginning(+1) *and* end of generation. ("==1" is required to monitor the outcome of the first iteration)
    {
        monitorPopulation();
    }
    
    // * Every N generations, take a video (duration: one generation time)
    
    if ( BoidsSharedData::gSnapshots )
    {
        if ( ( gWorld->getIterations() ) % ( BoidsSharedData::gEvaluationTime * BoidsSharedData::gSnapshotsFrequency ) == 0 )
        {
            if ( gVerbose )
                std::cout << "[START] Video recording: generation #" << (gWorld->getIterations() / BoidsSharedData::gEvaluationTime ) << ".\n";
            gTrajectoryMonitorMode = 0;
            initTrajectoriesMonitor();
        }
        else
            if ( ( gWorld->getIterations() ) % ( BoidsSharedData::gEvaluationTime * BoidsSharedData::gSnapshotsFrequency ) == BoidsSharedData::gEvaluationTime - 1 )
            {
                if ( gVerbose )
                    std::cout << "[STOP]  Video recording: generation #" << (gWorld->getIterations() / BoidsSharedData::gEvaluationTime ) << ".\n";
                saveTrajectoryImage();
            }
    }    
}

void BoidsWorldObserver::monitorPopulation( bool localVerbose )
{
    // * monitoring: count number of active agents.
    
    int activeCount = 0;
    for ( int i = 0 ; i != gNumberOfRobots ; i++ )
    {
        if ( (dynamic_cast<BoidsController*>(gWorld->getRobot(i)->getController()))->getWorldModel()->isAlive() == true )
            activeCount++;
    }
    
    if ( gVerbose && localVerbose )
    {
        std::cout << "[gen:" << (gWorld->getIterations()/BoidsSharedData::gEvaluationTime) << ";it:" << gWorld->getIterations() << ";pop:" << activeCount << "]\n";
    }
    
    // Logging, population-level: alive
    std::string sLog = std::string("") + std::to_string(gWorld->getIterations()) + ",pop,alive," + std::to_string(activeCount) + "\n";
    gLogManager->write(sLog);
    gLogManager->flush();
}