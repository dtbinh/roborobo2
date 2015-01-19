#if defined PRJ_POPGENMODELS || !defined MODULAR

#include "Config/PopGenModelsConfigurationLoader.h"

#include "PopGenModels/include/PopGenModelsWorldObserver.h"
#include "PopGenModels/include/PopGenModelsAgentObserver.h"
#include "PopGenModels/include/PopGenModelsController.h"

#include "WorldModels/RobotWorldModel.h"

PopGenModelsConfigurationLoader::PopGenModelsConfigurationLoader()
{
}

PopGenModelsConfigurationLoader::~PopGenModelsConfigurationLoader()
{
	//nothing to do
}

WorldObserver* PopGenModelsConfigurationLoader::make_WorldObserver(World* wm)
{
	return new PopGenModelsWorldObserver(wm);
}

RobotWorldModel* PopGenModelsConfigurationLoader::make_RobotWorldModel()
{
	return new RobotWorldModel();
}

AgentObserver* PopGenModelsConfigurationLoader::make_AgentObserver(RobotWorldModel* wm)
{
	return new PopGenModelsAgentObserver(wm);
}

Controller* PopGenModelsConfigurationLoader::make_Controller(RobotWorldModel* wm)
{
	return new PopGenModelsController(wm);
}

#endif
