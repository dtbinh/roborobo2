/**
 * @author Nicolas Bredeche <nicolas.bredeche@upmc.fr>
 * NNlib: Leo Cazenille <leo.cazenille@upmc.fr>
 */



#include "Boids/include/BoidsSharedData.h"


int BoidsSharedData::gEvaluationTime = 0; // how long a controller will be evaluated on a robot

double BoidsSharedData::gMonitorPositions;

bool BoidsSharedData::gPropertiesLoaded = false; // global variable local to file -- TODO: move specific properties loader in dedicated WorldObserver

bool BoidsSharedData::gSnapshots = true; // take snapshots
int BoidsSharedData::gSnapshotsFrequency = 50; // every N generations

