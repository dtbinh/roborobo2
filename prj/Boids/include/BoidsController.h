/**
 * @author Nicolas Bredeche <nicolas.bredeche@upmc.fr>
 * NNlib: Leo Cazenille <leo.cazenille@upmc.fr>
 */



#ifndef BOIDSCONTROLLER_H
#define BOIDSCONTROLLER_H

#include "RoboroboMain/common.h"
#include "RoboroboMain/roborobo.h"
#include "Utilities/Graphics.h"
#include "Controllers/Controller.h"
#include "WorldModels/RobotWorldModel.h"
#include "Boids/include/BoidsAgentObserver.h"
#include <neuralnetworks/NeuralNetwork.h>

#include <iomanip>

using namespace Neural;


class BoidsController : public Controller
{
private:
    int _iteration;
    void resetRobot();
    
public:
    
    BoidsController(RobotWorldModel *wm);
    ~BoidsController();
    
    void reset();
    void step();
};


#endif

