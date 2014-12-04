#if defined PRJ_BOIDS || !defined MODULAR

#include "Config/BoidsConfigurationLoader.h"

#include "Boids/include/BoidsWorldObserver.h"
#include "Boids/include/BoidsAgentObserver.h"
#include "Boids/include/BoidsController.h"

#include "WorldModels/RobotWorldModel.h"

BoidsConfigurationLoader::BoidsConfigurationLoader()
{
}

BoidsConfigurationLoader::~BoidsConfigurationLoader()
{
	//nothing to do
}

WorldObserver* BoidsConfigurationLoader::make_WorldObserver(World* wm)
{
	return new BoidsWorldObserver(wm);
}

RobotWorldModel* BoidsConfigurationLoader::make_RobotWorldModel()
{
	return new RobotWorldModel();
}

AgentObserver* BoidsConfigurationLoader::make_AgentObserver(RobotWorldModel* wm)
{
	return new BoidsAgentObserver(wm);
}

Controller* BoidsConfigurationLoader::make_Controller(RobotWorldModel* wm)
{
	return new BoidsController(wm);
}

#endif
