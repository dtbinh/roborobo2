/**
 * @author Nicolas Bredeche <nicolas.bredeche@upmc.fr>
 * NNlib: Leo Cazenille <leo.cazenille@upmc.fr>
 */


#ifndef POPGENMODELSAGENTOBSERVER_H
#define POPGENMODELSAGENTOBSERVER_H

#include "RoboroboMain/common.h"
#include "RoboroboMain/roborobo.h"
#include "Observers/AgentObserver.h"
#include "WorldModels/RobotWorldModel.h"
#include "PopGenModels/include/PopGenModelsSharedData.h"

#include <iomanip>

class PopGenModelsAgentObserver : public AgentObserver
{
	public:
		PopGenModelsAgentObserver(RobotWorldModel *wm);
		~PopGenModelsAgentObserver();

		void reset();
		void step();

};

#endif

