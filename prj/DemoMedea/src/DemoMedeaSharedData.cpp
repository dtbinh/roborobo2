/**
 * @author Nicolas Bredeche <nicolas.bredeche@upmc.fr>
 * NNlib: Leo Cazenille <leo.cazenille@upmc.fr>
 */



#include "DemoMedea/include/DemoMedeaSharedData.h"

double DemoMedeaSharedData::gSigmaMin = 0.0;
double DemoMedeaSharedData::gProbaMutation = 0.0;
double DemoMedeaSharedData::gUpdateSigmaStep = 0.0;
double DemoMedeaSharedData::gSigmaRef = 0.0; // reference value of sigma
double DemoMedeaSharedData::gSigmaMax = 0.0; // maximal value of sigma
int DemoMedeaSharedData::gEvaluationTime = 0; // how long a controller will be evaluated on a robot

bool DemoMedeaSharedData::gSynchronization = true;

bool DemoMedeaSharedData::gEnergyRequestOutput = 1;

double DemoMedeaSharedData::gMonitorPositions;

bool DemoMedeaSharedData::gPropertiesLoaded = false; // global variable local to file -- TODO: move specific properties loader in dedicated WorldObserver

int DemoMedeaSharedData::gNbHiddenLayers = 1;
int DemoMedeaSharedData::gNbNeuronsPerHiddenLayer = 5;
int DemoMedeaSharedData::gNeuronWeightRange = 800;

bool DemoMedeaSharedData::gSnapshots = true; // take snapshots
int DemoMedeaSharedData::gSnapshotsFrequency = 50; // every N generations

int DemoMedeaSharedData::gControllerType = -1; // cf. header for description

bool DemoMedeaSharedData::gLimitGenomeTransmission = false; // default: do not limit.
int DemoMedeaSharedData::gMaxNbGenomeTransmission = 65535; // default: arbitrarily set to 65535.

int DemoMedeaSharedData::gSelectionMethod = 0; // default: random selection

int DemoMedeaSharedData::gNotListeningStateDelay = 0;    // -1: infinite ; 0: no delay ; >0: delay
int DemoMedeaSharedData::gListeningStateDelay = -1;      // -1: infinite ; 0: no delay ; >0: delay (ignored if gNotListeningStateDelay=-1)

bool DemoMedeaSharedData::gLogGenome = false;

double DemoMedeaSharedData::gIndividualMutationRate = 1.0;

int DemoMedeaSharedData::gMutationOperator = 1; // 0: uniform, 1: gaussian

double DemoMedeaSharedData::gSigma = 0.01; // 0.01 is just some random value.