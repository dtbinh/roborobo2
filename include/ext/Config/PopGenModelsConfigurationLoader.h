/*
 * MedeaConfigurationLoader.h
 */

#ifndef POPGENMODELSCONFIGURATIONLOADER_H
#define POPGENMODELSCONFIGURATIONLOADER_H

#include "Config/ConfigurationLoader.h"


class PopGenModelsConfigurationLoader : public ConfigurationLoader
{
	private:

	public:
		PopGenModelsConfigurationLoader();
		~PopGenModelsConfigurationLoader();

		WorldObserver *make_WorldObserver(World* wm) ;
		RobotWorldModel *make_RobotWorldModel();
		AgentObserver *make_AgentObserver(RobotWorldModel* wm) ;
		Controller *make_Controller(RobotWorldModel* wm) ;
};



#endif
