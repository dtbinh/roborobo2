/**
 * @author Nicolas Bredeche <nicolas.bredeche@upmc.fr>
 * NNlib: Leo Cazenille <leo.cazenille@upmc.fr>
 */


#ifndef MEDEASPECIALIZATIONAGENTOBSERVER_H
#define MEDEASPECIALIZATIONAGENTOBSERVER_H

#include "RoboroboMain/common.h"
#include "RoboroboMain/roborobo.h"
#include "Observers/AgentObserver.h"
#include "WorldModels/RobotWorldModel.h"
#include "MedeaSpecialization/include/MedeaSpecializationSharedData.h"

#include <iomanip>

class MedeaSpecializationAgentObserver : public AgentObserver
{
	public:
		MedeaSpecializationAgentObserver(RobotWorldModel *wm);
		~MedeaSpecializationAgentObserver();

		void reset();
		void step();

};

#endif

