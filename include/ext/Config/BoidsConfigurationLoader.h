/*
 * MedeaConfigurationLoader.h
 */

#ifndef BOIDSCONFIGURATIONLOADER_H
#define BOIDSCONFIGURATIONLOADER_H

#include "Config/ConfigurationLoader.h"


class BoidsConfigurationLoader : public ConfigurationLoader
{
	private:

	public:
		BoidsConfigurationLoader();
		~BoidsConfigurationLoader();

		WorldObserver *make_WorldObserver(World* wm) ;
		RobotWorldModel *make_RobotWorldModel();
		AgentObserver *make_AgentObserver(RobotWorldModel* wm) ;
		Controller *make_Controller(RobotWorldModel* wm) ;
};



#endif
