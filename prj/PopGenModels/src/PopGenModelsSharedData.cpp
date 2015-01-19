/**
 * @author Nicolas Bredeche <nicolas.bredeche@upmc.fr>
 * NNlib: Leo Cazenille <leo.cazenille@upmc.fr>
 */



#include "PopGenModels/include/PopGenModelsSharedData.h"

double PopGenModelsSharedData::gSigmaMin = 0.0;
double PopGenModelsSharedData::gProbaMutation = 0.0;
double PopGenModelsSharedData::gUpdateSigmaStep = 0.0;
double PopGenModelsSharedData::gSigmaRef = 0.0; // reference value of sigma
double PopGenModelsSharedData::gSigmaMax = 0.0; // maximal value of sigma
int PopGenModelsSharedData::gEvaluationTime = 0; // how long a controller will be evaluated on a robot

bool PopGenModelsSharedData::gSynchronization = true;

bool PopGenModelsSharedData::gEnergyRequestOutput = 1;

double PopGenModelsSharedData::gMonitorPositions;

bool PopGenModelsSharedData::gPropertiesLoaded = false; // global variable local to file -- TODO: move specific properties loader in dedicated WorldObserver

int PopGenModelsSharedData::gNbHiddenLayers = 1;
int PopGenModelsSharedData::gNbNeuronsPerHiddenLayer = 5;
int PopGenModelsSharedData::gNeuronWeightRange = 800;

bool PopGenModelsSharedData::gSnapshots = true; // take snapshots
int PopGenModelsSharedData::gSnapshotsFrequency = 50; // every N generations

int PopGenModelsSharedData::gControllerType = -1; // cf. header for description

bool PopGenModelsSharedData::gLimitGenomeTransmission = false; // default: do not limit.
int PopGenModelsSharedData::gMaxNbGenomeTransmission = 65535; // default: arbitrarily set to 65535.

int PopGenModelsSharedData::gSelectionMethod = 0; // default: random selection

int PopGenModelsSharedData::gNotListeningStateDelay = 0;    // -1: infinite ; 0: no delay ; >0: delay
int PopGenModelsSharedData::gListeningStateDelay = -1;      // -1: infinite ; 0: no delay ; >0: delay (ignored if gNotListeningStateDelay=-1)

bool PopGenModelsSharedData::gLogGenome = false;

double PopGenModelsSharedData::gIndividualMutationRate = 0.1; // defaut value