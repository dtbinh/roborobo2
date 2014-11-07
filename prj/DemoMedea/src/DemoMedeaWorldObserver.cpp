/**
 * @author Nicolas Bredeche <nicolas.bredeche@upmc.fr>
 * NNlib: Leo Cazenille <leo.cazenille@upmc.fr>
 */

#include "Observers/AgentObserver.h"
#include "Observers/WorldObserver.h"
#include "DemoMedea/include/DemoMedeaWorldObserver.h"
#include "DemoMedea/include/DemoMedeaController.h"
#include "World/World.h"


DemoMedeaWorldObserver::DemoMedeaWorldObserver( World* world ) : WorldObserver( world )
{
    _world = world;
    
    // ==== loading project-specific properties
    
    gProperties.checkAndGetPropertyValue("gSigmaRef",&DemoMedeaSharedData::gSigmaRef,true);
    gProperties.checkAndGetPropertyValue("gSigmaMin",&DemoMedeaSharedData::gSigmaMin,true);
    gProperties.checkAndGetPropertyValue("gSigmaMax",&DemoMedeaSharedData::gSigmaMax,true);
    
    gProperties.checkAndGetPropertyValue("gProbaMutation",&DemoMedeaSharedData::gProbaMutation,true);
    gProperties.checkAndGetPropertyValue("gUpdateSigmaStep",&DemoMedeaSharedData::gUpdateSigmaStep,true);
    gProperties.checkAndGetPropertyValue("gEvaluationTime",&DemoMedeaSharedData::gEvaluationTime,true);
    gProperties.checkAndGetPropertyValue("gSynchronization",&DemoMedeaSharedData::gSynchronization,true);
    
    gProperties.checkAndGetPropertyValue("gEnergyRequestOutput",&DemoMedeaSharedData::gEnergyRequestOutput,false);
    
    gProperties.checkAndGetPropertyValue("gMonitorPositions",&DemoMedeaSharedData::gMonitorPositions,true);
    
    gProperties.checkAndGetPropertyValue("gNbHiddenLayers",&DemoMedeaSharedData::gNbHiddenLayers,true);
    gProperties.checkAndGetPropertyValue("gNbNeuronsPerHiddenLayer",&DemoMedeaSharedData::gNbNeuronsPerHiddenLayer,true);
    gProperties.checkAndGetPropertyValue("gNeuronWeightRange",&DemoMedeaSharedData::gNeuronWeightRange,true);
    
    gProperties.checkAndGetPropertyValue("gSnapshots",&DemoMedeaSharedData::gSnapshots,false);
    gProperties.checkAndGetPropertyValue("gSnapshotsFrequency",&DemoMedeaSharedData::gSnapshotsFrequency,false);
    
    gProperties.checkAndGetPropertyValue("gControllerType",&DemoMedeaSharedData::gControllerType,true);
    
    gProperties.checkAndGetPropertyValue("gMaxNbGenomeTransmission",&DemoMedeaSharedData::gMaxNbGenomeTransmission,true);
    gProperties.checkAndGetPropertyValue("gLimitGenomeTransmission",&DemoMedeaSharedData::gLimitGenomeTransmission,true);
    gProperties.checkAndGetPropertyValue("gSelectionMethod",&DemoMedeaSharedData::gSelectionMethod,true);
    
    gProperties.checkAndGetPropertyValue("gNotListeningStateDelay",&DemoMedeaSharedData::gNotListeningStateDelay,true);
    gProperties.checkAndGetPropertyValue("gListeningStateDelay",&DemoMedeaSharedData::gListeningStateDelay,true);
    
    
    // ====
    
    if ( !gRadioNetwork)
    {
        std::cout << "Error : gRadioNetwork must be true." << std::endl;
        exit(-1);
    }
    
    // * iteration and generation counters
    
    _lifeIterationCount = -1;
    _generationCount = -1;
    
}

DemoMedeaWorldObserver::~DemoMedeaWorldObserver()
{
    // nothing to do.
}

void DemoMedeaWorldObserver::reset()
{
    // nothing to do.
}

void DemoMedeaWorldObserver::step()
{
    _lifeIterationCount++;
    
    updateMonitoring();
    
    if( _lifeIterationCount >= DemoMedeaSharedData::gEvaluationTime ) // switch to next generation.
    {
        // update iterations and generations counters
        _lifeIterationCount = 0;
        _generationCount++;
    }
    
    updateEnvironment();
    
}


void DemoMedeaWorldObserver::updateEnvironment()
{
    // ...
}

void DemoMedeaWorldObserver::updateMonitoring()
{
    // * Log at end of each generation
    
    if( _lifeIterationCount >= DemoMedeaSharedData::gEvaluationTime ) // end of generation.
    {
        // * monitoring: count number of active agents.
        
        int activeCount = 0;
        for ( int i = 0 ; i != gNumberOfRobots ; i++ )
        {
            if ( (dynamic_cast<DemoMedeaController*>(gWorld->getRobot(i)->getController()))->getWorldModel()->isAlive() == true )
                activeCount++;
        }
        
        if ( gVerbose )
        {
            std::cout << "[gen:" << (gWorld->getIterations()/DemoMedeaSharedData::gEvaluationTime) << ";pop:" << activeCount << "]\n";
        }
        
        // Logging, population-level: alive
        std::string sLog = std::string("") + "{" + std::to_string(gWorld->getIterations()) + "}[all] [pop_alive:" + std::to_string(activeCount) + "]\n";
        gLogManager->write(sLog);
        gLogManager->flush();
    }
    
    // * Every N generations, take a video (duration: one generation time)
    
    if ( DemoMedeaSharedData::gSnapshots )
    {
        if ( ( gWorld->getIterations() ) % ( DemoMedeaSharedData::gEvaluationTime * DemoMedeaSharedData::gSnapshotsFrequency ) == 0 )
        {
            if ( gVerbose )
                std::cout << "[START] Video recording: generation #" << (gWorld->getIterations() / DemoMedeaSharedData::gEvaluationTime ) << ".\n";
            gTrajectoryMonitorMode = 0;
            initTrajectoriesMonitor();
        }
        else
            if ( ( gWorld->getIterations() ) % ( DemoMedeaSharedData::gEvaluationTime * DemoMedeaSharedData::gSnapshotsFrequency ) == DemoMedeaSharedData::gEvaluationTime - 1 )
            {
                if ( gVerbose )
                    std::cout << "[STOP]  Video recording: generation #" << (gWorld->getIterations() / DemoMedeaSharedData::gEvaluationTime ) << ".\n";
                saveTrajectoryImage();
            }
    }    
}

