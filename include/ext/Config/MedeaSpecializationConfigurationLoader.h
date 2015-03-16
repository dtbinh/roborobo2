/*
 * MedeaConfigurationLoader.h
 */

#ifndef MEDEASPECIALIZATIONCONFIGURATIONLOADER_H
#define MEDEASPECIALIZATIONCONFIGURATIONLOADER_H

#include "Config/ConfigurationLoader.h"


class MedeaSpecializationConfigurationLoader : public ConfigurationLoader
{
	private:

	public:
		MedeaSpecializationConfigurationLoader();
		~MedeaSpecializationConfigurationLoader();

		WorldObserver *make_WorldObserver(World* wm) ;
		RobotWorldModel *make_RobotWorldModel();
		AgentObserver *make_AgentObserver(RobotWorldModel* wm) ;
		Controller *make_Controller(RobotWorldModel* wm) ;
};



#endif
