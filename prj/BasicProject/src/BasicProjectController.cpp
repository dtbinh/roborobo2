/**
 * @author Nicolas Bredeche <nicolas.bredeche@upmc.fr>
 */


#include "BasicProject/include/BasicProjectController.h"

BasicProjectController::BasicProjectController( RobotWorldModel *__wm ) : Controller ( __wm )
{
	// nothing to do
}

BasicProjectController::~BasicProjectController()
{
	// nothing to do.
}

void BasicProjectController::reset()
{
	// nothing to do.
}

void BasicProjectController::step()
{
	// a basic obstacle avoidance behavior

	_wm->_desiredTranslationalValue =  + 1 - ( (double)gSensorRange - ((_wm->getCameraSensorValue(2,SENSOR_DISTANCEVALUE)+_wm->getCameraSensorValue(3,SENSOR_DISTANCEVALUE))/2) )  / (double)gSensorRange;
	if ( _wm->getCameraSensorValue(0,SENSOR_DISTANCEVALUE) + _wm->getCameraSensorValue(1,SENSOR_DISTANCEVALUE) + _wm->getCameraSensorValue(2,SENSOR_DISTANCEVALUE) < _wm->getCameraSensorValue(3,SENSOR_DISTANCEVALUE) + _wm->getCameraSensorValue(4,SENSOR_DISTANCEVALUE) + _wm->getCameraSensorValue(5,SENSOR_DISTANCEVALUE) )
		_wm->_desiredRotationalVelocity = +5;
	else
		if ( _wm->getCameraSensorValue(3,SENSOR_DISTANCEVALUE) + _wm->getCameraSensorValue(4,SENSOR_DISTANCEVALUE) + _wm->getCameraSensorValue(5,SENSOR_DISTANCEVALUE) < 3*gSensorRange )
			_wm->_desiredRotationalVelocity = -5;
		else
			if ( _wm->_desiredRotationalVelocity > 0 ) 
				_wm->_desiredRotationalVelocity--;
			else
				if ( _wm->_desiredRotationalVelocity < 0) 
					_wm->_desiredRotationalVelocity++;
				else
					_wm->_desiredRotationalVelocity = 0.01 - (double)(rand()%10)/10.*0.02;
}

