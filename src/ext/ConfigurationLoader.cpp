#include "Config/ConfigurationLoader.h"
#include <string.h>

#include "Config/BasicProjectConfigurationLoader.h"
#include "Config/DemoMedeaConfigurationLoader.h"
#include "Config/BoidsConfigurationLoader.h"
#include "Config/PopGenModelsConfigurationLoader.h"
#include "Config/MedeaSpecializationConfigurationLoader.h"
//###DO-NOT-DELETE-THIS-LINE###TAG:INCLUDE###//


ConfigurationLoader::ConfigurationLoader()
{
	//nothing to do
}

ConfigurationLoader::~ConfigurationLoader()
{
	//nothing to do
}

ConfigurationLoader* ConfigurationLoader::make_ConfigurationLoader (std::string configurationLoaderObjectName)
{
	if (0)
	{
		// >>> Never reached
	}
#if defined PRJ_BASICPROJECT || !defined MODULAR
    else if (configurationLoaderObjectName == "BasicProjectConfigurationLoader" )
    {
        return new BasicProjectConfigurationLoader();
    }
#endif
#if defined PRJ_DEMOMEDEA || !defined MODULAR
	else if (configurationLoaderObjectName == "DemoMedeaConfigurationLoader" )
	{
		return new DemoMedeaConfigurationLoader();
	}
#endif
#if defined PRJ_BOIDS || !defined MODULAR
	else if (configurationLoaderObjectName == "BoidsConfigurationLoader" )
	{
		return new BoidsConfigurationLoader();
	}
#endif
#if defined PRJ_POPGENMODELS || !defined MODULAR
	else if (configurationLoaderObjectName == "PopGenModelsConfigurationLoader" )
	{
		return new PopGenModelsConfigurationLoader();
	}
#endif
#if defined PRJ_MEDEASPECIALIZATION || !defined MODULAR
	else if (configurationLoaderObjectName == "MedeaSpecializationConfigurationLoader" )
	{
		return new MedeaSpecializationConfigurationLoader();
	}
#endif
    //###DO-NOT-DELETE-THIS-LINE###TAG:SWITCH###//
	else
	{
		return NULL;
	}

}
