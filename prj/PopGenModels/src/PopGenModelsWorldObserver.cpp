/**
 * @author Nicolas Bredeche <nicolas.bredeche@upmc.fr>
 * NNlib: Leo Cazenille <leo.cazenille@upmc.fr>
 */

#include "Observers/AgentObserver.h"
#include "Observers/WorldObserver.h"
#include "PopGenModels/include/PopGenModelsWorldObserver.h"
#include "PopGenModels/include/PopGenModelsController.h"
#include "World/World.h"


PopGenModelsWorldObserver::PopGenModelsWorldObserver( World* world ) : WorldObserver( world )
{
    _world = world;
    
    // ==== loading project-specific properties
    
    gProperties.checkAndGetPropertyValue("gSigmaRef",&PopGenModelsSharedData::gSigmaRef,true);
    gProperties.checkAndGetPropertyValue("gSigmaMin",&PopGenModelsSharedData::gSigmaMin,true);
    gProperties.checkAndGetPropertyValue("gSigmaMax",&PopGenModelsSharedData::gSigmaMax,true);
    
    gProperties.checkAndGetPropertyValue("gProbaMutation",&PopGenModelsSharedData::gProbaMutation,true);
    gProperties.checkAndGetPropertyValue("gUpdateSigmaStep",&PopGenModelsSharedData::gUpdateSigmaStep,true);
    gProperties.checkAndGetPropertyValue("gEvaluationTime",&PopGenModelsSharedData::gEvaluationTime,true);
    gProperties.checkAndGetPropertyValue("gSynchronization",&PopGenModelsSharedData::gSynchronization,true);
    
    gProperties.checkAndGetPropertyValue("gEnergyRequestOutput",&PopGenModelsSharedData::gEnergyRequestOutput,false);
    
    gProperties.checkAndGetPropertyValue("gMonitorPositions",&PopGenModelsSharedData::gMonitorPositions,true);
    
    gProperties.checkAndGetPropertyValue("gNbHiddenLayers",&PopGenModelsSharedData::gNbHiddenLayers,true);
    gProperties.checkAndGetPropertyValue("gNbNeuronsPerHiddenLayer",&PopGenModelsSharedData::gNbNeuronsPerHiddenLayer,true);
    gProperties.checkAndGetPropertyValue("gNeuronWeightRange",&PopGenModelsSharedData::gNeuronWeightRange,true);
    
    gProperties.checkAndGetPropertyValue("gSnapshots",&PopGenModelsSharedData::gSnapshots,false);
    gProperties.checkAndGetPropertyValue("gSnapshotsFrequency",&PopGenModelsSharedData::gSnapshotsFrequency,false);
    
    gProperties.checkAndGetPropertyValue("gControllerType",&PopGenModelsSharedData::gControllerType,true);
    
    gProperties.checkAndGetPropertyValue("gMaxNbGenomeTransmission",&PopGenModelsSharedData::gMaxNbGenomeTransmission,true);
    gProperties.checkAndGetPropertyValue("gLimitGenomeTransmission",&PopGenModelsSharedData::gLimitGenomeTransmission,true);
    gProperties.checkAndGetPropertyValue("gSelectionMethod",&PopGenModelsSharedData::gSelectionMethod,true);
    
    gProperties.checkAndGetPropertyValue("gNotListeningStateDelay",&PopGenModelsSharedData::gNotListeningStateDelay,true);
    gProperties.checkAndGetPropertyValue("gListeningStateDelay",&PopGenModelsSharedData::gListeningStateDelay,true);
    
    gProperties.checkAndGetPropertyValue("gLogGenome",&PopGenModelsSharedData::gLogGenome,false);
    
    gProperties.checkAndGetPropertyValue("gIndividualMutationRate",&PopGenModelsSharedData::gIndividualMutationRate,false);
    
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

PopGenModelsWorldObserver::~PopGenModelsWorldObserver()
{
    // nothing to do.
}

void PopGenModelsWorldObserver::reset()
{
    // nothing to do.
}

void PopGenModelsWorldObserver::step()
{
    _generationItCount++;
    
    if( _generationItCount == PopGenModelsSharedData::gEvaluationTime+1 ) // switch to next generation.
    {
        // update iterations and generations counters
        _generationItCount = 0;
        _generationCount++;
    }
    
    updateMonitoring();
    
    updateEnvironment();
    
}


void PopGenModelsWorldObserver::updateEnvironment()
{
    // ...
}

void PopGenModelsWorldObserver::updateMonitoring()
{
    // * Log at end of each generation

    if( gWorld->getIterations() % PopGenModelsSharedData::gEvaluationTime == 1 || gWorld->getIterations() % PopGenModelsSharedData::gEvaluationTime == PopGenModelsSharedData::gEvaluationTime-1 ) // beginning(+1) *and* end of generation. ("==1" is required to monitor the outcome of the first iteration)
    {
        monitorPopulation();
    }
    
    // * Every N generations, take a video (duration: one generation time)
    
    if ( PopGenModelsSharedData::gSnapshots )
    {
        if ( ( gWorld->getIterations() ) % ( PopGenModelsSharedData::gEvaluationTime * PopGenModelsSharedData::gSnapshotsFrequency ) == 0 )
        {
            if ( gVerbose )
                std::cout << "[START] Video recording: generation #" << (gWorld->getIterations() / PopGenModelsSharedData::gEvaluationTime ) << ".\n";
            gTrajectoryMonitorMode = 0;
            initTrajectoriesMonitor();
        }
        else
            if ( ( gWorld->getIterations() ) % ( PopGenModelsSharedData::gEvaluationTime * PopGenModelsSharedData::gSnapshotsFrequency ) == PopGenModelsSharedData::gEvaluationTime - 1 )
            {
                if ( gVerbose )
                    std::cout << "[STOP]  Video recording: generation #" << (gWorld->getIterations() / PopGenModelsSharedData::gEvaluationTime ) << ".\n";
                saveTrajectoryImage();
            }
    }    
}

void PopGenModelsWorldObserver::monitorPopulation( bool localVerbose )
{
    // * monitoring: count number of active agents.
    
    int activeCount = 0;
    for ( int i = 0 ; i != gNumberOfRobots ; i++ )
    {
        if ( (dynamic_cast<PopGenModelsController*>(gWorld->getRobot(i)->getController()))->getWorldModel()->isAlive() == true )
            activeCount++;
    }
    
    if ( gVerbose && localVerbose )
    {
        std::cout << "[gen:" << (gWorld->getIterations()/PopGenModelsSharedData::gEvaluationTime) << ";it:" << gWorld->getIterations() << ";pop:" << activeCount << "]\n";
    }
    
    // Logging, population-level: alive
    std::string sLog = std::string("") + std::to_string(gWorld->getIterations()) + ",pop,alive," + std::to_string(activeCount) + "\n";
    gLogManager->write(sLog);
    gLogManager->flush();
}