/**
 * @author Nicolas Bredeche <nicolas.bredeche@upmc.fr>
 * NNlib: Leo Cazenille <leo.cazenille@upmc.fr>
 */


#ifndef BOIDSAGENTOBSERVER_H
#define BOIDSAGENTOBSERVER_H

#include "RoboroboMain/common.h"
#include "RoboroboMain/roborobo.h"
#include "Observers/AgentObserver.h"
#include "WorldModels/RobotWorldModel.h"
#include "Boids/include/BoidsSharedData.h"

#include <iomanip>

class BoidsAgentObserver : public AgentObserver
{
	public:
		BoidsAgentObserver(RobotWorldModel *wm);
		~BoidsAgentObserver();

		void reset();
		void step();

};

#endif

