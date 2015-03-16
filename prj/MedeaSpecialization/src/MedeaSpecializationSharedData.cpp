/**
 * @author Nicolas Bredeche <nicolas.bredeche@upmc.fr>
 * NNlib: Leo Cazenille <leo.cazenille@upmc.fr>
 */



#include "MedeaSpecialization/include/MedeaSpecializationSharedData.h"

double MedeaSpecializationSharedData::gSigmaMin = 0.0;
double MedeaSpecializationSharedData::gProbaMutation = 0.0;
double MedeaSpecializationSharedData::gUpdateSigmaStep = 0.0;
double MedeaSpecializationSharedData::gSigmaRef = 0.0; // reference value of sigma
double MedeaSpecializationSharedData::gSigmaMax = 0.0; // maximal value of sigma
int MedeaSpecializationSharedData::gEvaluationTime = 0; // how long a controller will be evaluated on a robot

bool MedeaSpecializationSharedData::gSynchronization = true;

bool MedeaSpecializationSharedData::gEnergyRequestOutput = 1;

double MedeaSpecializationSharedData::gMonitorPositions;

bool MedeaSpecializationSharedData::gPropertiesLoaded = false; // global variable local to file -- TODO: move specific properties loader in dedicated WorldObserver

int MedeaSpecializationSharedData::gNbHiddenLayers = 1;
int MedeaSpecializationSharedData::gNbNeuronsPerHiddenLayer = 5;
int MedeaSpecializationSharedData::gNeuronWeightRange = 800;

bool MedeaSpecializationSharedData::gSnapshots = true; // take snapshots
int MedeaSpecializationSharedData::gSnapshotsFrequency = 50; // every N generations

int MedeaSpecializationSharedData::gControllerType = -1; // cf. header for description

bool MedeaSpecializationSharedData::gLimitGenomeTransmission = false; // default: do not limit.
int MedeaSpecializationSharedData::gMaxNbGenomeTransmission = 65535; // default: arbitrarily set to 65535.

int MedeaSpecializationSharedData::gSelectionMethod = 0; // default: random selection

int MedeaSpecializationSharedData::gNotListeningStateDelay = 0;    // -1: infinite ; 0: no delay ; >0: delay
int MedeaSpecializationSharedData::gListeningStateDelay = -1;      // -1: infinite ; 0: no delay ; >0: delay (ignored if gNotListeningStateDelay=-1)

bool MedeaSpecializationSharedData::gLogGenome = false;

double MedeaSpecializationSharedData::gIndividualMutationRate = 1.0;

int MedeaSpecializationSharedData::gMutationOperator = 1; // 0: uniform, 1: gaussian

double MedeaSpecializationSharedData::gSigma = 0.01; // 0.01 is just some random value.