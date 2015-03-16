#if defined PRJ_MEDEASPECIALIZATION || !defined MODULAR

#include "Config/MedeaSpecializationConfigurationLoader.h"

#include "MedeaSpecialization/include/MedeaSpecializationWorldObserver.h"
#include "MedeaSpecialization/include/MedeaSpecializationAgentObserver.h"
#include "MedeaSpecialization/include/MedeaSpecializationController.h"

#include "WorldModels/RobotWorldModel.h"

MedeaSpecializationConfigurationLoader::MedeaSpecializationConfigurationLoader()
{
}

MedeaSpecializationConfigurationLoader::~MedeaSpecializationConfigurationLoader()
{
	//nothing to do
}

WorldObserver* MedeaSpecializationConfigurationLoader::make_WorldObserver(World* wm)
{
	return new MedeaSpecializationWorldObserver(wm);
}

RobotWorldModel* MedeaSpecializationConfigurationLoader::make_RobotWorldModel()
{
	return new RobotWorldModel();
}

AgentObserver* MedeaSpecializationConfigurationLoader::make_AgentObserver(RobotWorldModel* wm)
{
	return new MedeaSpecializationAgentObserver(wm);
}

Controller* MedeaSpecializationConfigurationLoader::make_Controller(RobotWorldModel* wm)
{
	return new MedeaSpecializationController(wm);
}

#endif
