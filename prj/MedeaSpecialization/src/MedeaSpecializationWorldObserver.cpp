/**
 * @author Nicolas Bredeche <nicolas.bredeche@upmc.fr>
 * NNlib: Leo Cazenille <leo.cazenille@upmc.fr>
 */

#include "Observers/AgentObserver.h"
#include "Observers/WorldObserver.h"
#include "MedeaSpecialization/include/MedeaSpecializationWorldObserver.h"
#include "MedeaSpecialization/include/MedeaSpecializationController.h"
#include "World/World.h"

MedeaSpecializationWorldObserver::MedeaSpecializationWorldObserver( World* world ) : WorldObserver( world )
{
    _world = world;
    
    // ==== loading project-specific properties
    
    gProperties.checkAndGetPropertyValue("gSigmaRef",&MedeaSpecializationSharedData::gSigmaRef,true);
    gProperties.checkAndGetPropertyValue("gSigmaMin",&MedeaSpecializationSharedData::gSigmaMin,true);
    gProperties.checkAndGetPropertyValue("gSigmaMax",&MedeaSpecializationSharedData::gSigmaMax,true);
    
    gProperties.checkAndGetPropertyValue("gProbaMutation",&MedeaSpecializationSharedData::gProbaMutation,true);
    gProperties.checkAndGetPropertyValue("gUpdateSigmaStep",&MedeaSpecializationSharedData::gUpdateSigmaStep,true);
    gProperties.checkAndGetPropertyValue("gEvaluationTime",&MedeaSpecializationSharedData::gEvaluationTime,true);
    gProperties.checkAndGetPropertyValue("gSynchronization",&MedeaSpecializationSharedData::gSynchronization,true);
    
    gProperties.checkAndGetPropertyValue("gEnergyRequestOutput",&MedeaSpecializationSharedData::gEnergyRequestOutput,false);
    
    gProperties.checkAndGetPropertyValue("gMonitorPositions",&MedeaSpecializationSharedData::gMonitorPositions,true);
    
    gProperties.checkAndGetPropertyValue("gNbHiddenLayers",&MedeaSpecializationSharedData::gNbHiddenLayers,true);
    gProperties.checkAndGetPropertyValue("gNbNeuronsPerHiddenLayer",&MedeaSpecializationSharedData::gNbNeuronsPerHiddenLayer,true);
    gProperties.checkAndGetPropertyValue("gNeuronWeightRange",&MedeaSpecializationSharedData::gNeuronWeightRange,true);
    
    gProperties.checkAndGetPropertyValue("gSnapshots",&MedeaSpecializationSharedData::gSnapshots,false);
    gProperties.checkAndGetPropertyValue("gSnapshotsFrequency",&MedeaSpecializationSharedData::gSnapshotsFrequency,false);
    
    gProperties.checkAndGetPropertyValue("gControllerType",&MedeaSpecializationSharedData::gControllerType,true);
    
    gProperties.checkAndGetPropertyValue("gMaxNbGenomeTransmission",&MedeaSpecializationSharedData::gMaxNbGenomeTransmission,true);
    gProperties.checkAndGetPropertyValue("gLimitGenomeTransmission",&MedeaSpecializationSharedData::gLimitGenomeTransmission,true);
    gProperties.checkAndGetPropertyValue("gSelectionMethod",&MedeaSpecializationSharedData::gSelectionMethod,true);
    
    gProperties.checkAndGetPropertyValue("gNotListeningStateDelay",&MedeaSpecializationSharedData::gNotListeningStateDelay,true);
    gProperties.checkAndGetPropertyValue("gListeningStateDelay",&MedeaSpecializationSharedData::gListeningStateDelay,true);
    
    gProperties.checkAndGetPropertyValue("gLogGenome",&MedeaSpecializationSharedData::gLogGenome,false);
    
    gProperties.checkAndGetPropertyValue("gIndividualMutationRate",&MedeaSpecializationSharedData::gIndividualMutationRate,false);

    gProperties.checkAndGetPropertyValue("gMutationOperator",&MedeaSpecializationSharedData::gMutationOperator,false);
    
    gProperties.checkAndGetPropertyValue("gSigma",&MedeaSpecializationSharedData::gSigma,false);

    
    // ====
    
    if ( !gRadioNetwork)
    {
        std::cout << "Error : gRadioNetwork must be true." << std::endl;
        exit(-1);
    }
    
    // * iteration and generation counters
    
    _generationItCount = -1;
    _generationCount = -1;
    
    
    // * list of position for landmark(s)
    
    int nbPos = 8;
    
    //for(int i=0; i<nbPos; ++i){positions.push_back(Point2d(rand()%960+20,rand()%960+20));}
    
    positions.push_back(Point2d(500,100));
    positions.push_back(Point2d(220,220));
    positions.push_back(Point2d(100,500));
    positions.push_back(Point2d(220,780));
    positions.push_back(Point2d(500,900));
    positions.push_back(Point2d(780,780));
    positions.push_back(Point2d(900,500));
    positions.push_back(Point2d(780,220));
    
    posIndex_landmark1 = 0;
    posIndex_landmark2 = nbPos / 2;

    
}

MedeaSpecializationWorldObserver::~MedeaSpecializationWorldObserver()
{
    // nothing to do.
}

void MedeaSpecializationWorldObserver::reset()
{
    if ( gLandmarks.size() == 2 )
    {
        gLandmarks[0].setColor(255, 128, 0, 0);
        gLandmarks[1].setColor(128, 255, 0, 0);
    }
    else
    {
        std::cout << "[ERROR] MedeaSpecialization project requires two landmarks. Exiting." << std::endl;
        exit (-1);
    }
}

void MedeaSpecializationWorldObserver::step()
{
    _generationItCount++;
    
    if( _generationItCount == MedeaSpecializationSharedData::gEvaluationTime+1 ) // switch to next generation.
    {
        // update iterations and generations counters
        _generationItCount = 0;
        _generationCount++;
    }
    
    updateMonitoring();
    
    updateEnvironment();
    
}


void MedeaSpecializationWorldObserver::updateEnvironment()
{
    
    // moving landmarks
    
    if ( gWorld->getIterations() % 400 == 0 )
    {
        gLandmarks[0].setPosition(positions[posIndex_landmark1]);
        gLandmarks[1].setPosition(positions[posIndex_landmark2]);
        posIndex_landmark1 = ( posIndex_landmark1 + 1 ) % positions.size();
        posIndex_landmark2 = ( posIndex_landmark2 + 1 ) % positions.size();
    }
}

void MedeaSpecializationWorldObserver::updateMonitoring()
{
    // * Log at end of each generation

    if( gWorld->getIterations() % MedeaSpecializationSharedData::gEvaluationTime == 1 || gWorld->getIterations() % MedeaSpecializationSharedData::gEvaluationTime == MedeaSpecializationSharedData::gEvaluationTime-1 ) // beginning(+1) *and* end of generation. ("==1" is required to monitor the outcome of the first iteration)
    {
        monitorPopulation();
    }
    
    // * Every N generations, take a video (duration: one generation time)
    
    if ( MedeaSpecializationSharedData::gSnapshots )
    {
        if ( ( gWorld->getIterations() ) % ( MedeaSpecializationSharedData::gEvaluationTime * MedeaSpecializationSharedData::gSnapshotsFrequency ) == 0 )
        {
            if ( gVerbose )
                std::cout << "[START] Video recording: generation #" << (gWorld->getIterations() / MedeaSpecializationSharedData::gEvaluationTime ) << ".\n";
            gTrajectoryMonitorMode = 0;
            initTrajectoriesMonitor();
        }
        else
            if ( ( gWorld->getIterations() ) % ( MedeaSpecializationSharedData::gEvaluationTime * MedeaSpecializationSharedData::gSnapshotsFrequency ) == MedeaSpecializationSharedData::gEvaluationTime - 1 )
            {
                if ( gVerbose )
                    std::cout << "[STOP]  Video recording: generation #" << (gWorld->getIterations() / MedeaSpecializationSharedData::gEvaluationTime ) << ".\n";
                saveTrajectoryImage();
            }
    }    
}

void MedeaSpecializationWorldObserver::monitorPopulation( bool localVerbose )
{
    // * monitoring: count number of active agents.
    
    int activeCount = 0;
    for ( int i = 0 ; i != gNumberOfRobots ; i++ )
    {
        if ( (dynamic_cast<MedeaSpecializationController*>(gWorld->getRobot(i)->getController()))->getWorldModel()->isAlive() == true )
            activeCount++;
    }
    
    if ( gVerbose && localVerbose )
    {
        std::cout << "[gen:" << (gWorld->getIterations()/MedeaSpecializationSharedData::gEvaluationTime) << ";it:" << gWorld->getIterations() << ";pop:" << activeCount << "]\n";
    }
    
    // Logging, population-level: alive
    std::string sLog = std::string("") + std::to_string(gWorld->getIterations()) + ",pop,alive," + std::to_string(activeCount) + "\n";
    gLogManager->write(sLog);
    gLogManager->flush();
}