/*
 *  SDL_gfxRoborobo.cpp
 *  roborobo
 *
 *  Created by Nicolas on 16/01/09.
 *  Copyright 2009 __MyCompanyName__. All rights reserved.
 *
 */

#include "Utilities/Graphics.h"
#include "Utilities/Misc.h"


int gScreenshotIndex = 0; // numbering screenshot (used by saveScreenshot(...))
int gRobotTrackerIndex = 0;
int gTrajectoryFileIndex = 0; // numbering trajectory images (used by saveTrajectoryImage(...))


void saveImage ( SDL_Surface *image, std::string __prefix, std::string __comment ) // comment is optional
{
    std::string sLog = gLogDirectoryname + "/" + __prefix + "_" + gStartTime;

	if ( __comment != "" )
		sLog += "_" + __comment;
	
	sLog += ".bmp";
	
	SDL_SaveBMP(image,sLog.c_str());
}

void saveTrajectoryImage ( std::string __comment )
{

    if ( gTrajectoryMonitor )
    {
        std::string trajectoryFileIndexStr = convertToString(gTrajectoryFileIndex);
        while( trajectoryFileIndexStr.length() < 6 )
        {
            trajectoryFileIndexStr =  "0" + trajectoryFileIndexStr;
        }
        
        std::string timestamp = convertToString(gTrajectoryMonitorIt);
        //while( timestamp.length() < 10 )
        //{
        //    timestamp =  "0" + timestamp;
        //}
        timestamp = "_it" + timestamp;

        std::string s = trajectoryFileIndexStr;
        
        if ( gTrajectoryMonitorMode == 0 )
            s += timestamp + "_all";
        else
            s += timestamp + "_single";
        
        if ( __comment != "" )
            s += "_" + __comment;
        
        saveImage(gTrajectoryMonitorImage,"trajectory",s);
        
        SDL_FreeSurface( gTrajectoryMonitorImage );
        gTrajectoryMonitor = false;
        
        gTrajectoryFileIndex++;
        
    }
    else
    {
        std::cout << "[WARNING] Cannot save trajectories. Monitor is not in use.\n";
    }
}

void saveScreenshot(std::string __comment) // parameter is optional
{
    std::string screenShotIndexStr = convertToString(gScreenshotIndex);
    
    while( screenShotIndexStr.length() < 6 )
    {
        screenShotIndexStr =  "0" + screenShotIndexStr;
    }
    
    saveImage(gScreen,"screenshot",screenShotIndexStr+"_"+__comment);

    gScreenshotIndex++;
}

void saveRobotTrackerIndex( std::string __comment) // parameter is optional
{
    std::string robotTrackerIndexStr = convertToString(gRobotTrackerIndex);
    
    while( robotTrackerIndexStr.length() < 6 )
    {
        robotTrackerIndexStr =  "0" + robotTrackerIndexStr;
    }
    
    saveImage(gEnvironmentImage,"robotsId",robotTrackerIndexStr+"_"+__comment);
    
	gRobotTrackerIndex++;
}

bool initSDL(Uint32 flags) // parameter is optional (default: SDL_HWSURFACE | SDL_DOUBLEBUF)
{
    
    if ( gBatchMode )
    {
            SDL_putenv((char*)"SDL_VIDEODRIVER=dummy"); // batch mode! (may not work with mac-os -- remove line)
            gDisplayMode = 0;
    }
	
    //Initialize all SDL subsystems
    if( SDL_Init( SDL_INIT_EVERYTHING ) < 0 )
    {
        return false;    
    }
	
	// set up screen
	gScreen = SDL_SetVideoMode( gScreenWidth, gScreenHeight, gScreenBPP, flags );

    //If there was an error in setting up the screen
    if( gScreen == NULL )
    {
        return false;    
    }
    
    //Set the window caption
    SDL_WM_SetCaption( "roborobo^2", NULL );
    
    //If everything initialized fine
    return true;
}

void apply_surface( int x, int y, SDL_Surface* source, SDL_Surface* destination , SDL_Rect* clip ) // last argument is optional (cf. function header)
{
    //Holds offsets
    SDL_Rect offset;
    
    //Get offsets
    offset.x = x;
    offset.y = y;
    
    //Blit
    SDL_BlitSurface( source, clip, destination, &offset );
}

void register_surface( int __x, int __y, SDL_Surface* source, SDL_Surface* destination , int __id ) // last argument is optional (cf. function header)
{
	for ( int x = 0 ; x != source->w ; x++ )
		for ( int y = 0 ; y != source->h ; y++ )
		{
			Uint32 pixel = getPixel32(source,x,y);
			if ( pixel != SDL_MapRGBA( destination->format, 0xFF, 0xFF, 0xFF, 0 ) )
				//putPixel32( destination, __x+x, __y+y, __id );
				putPixel32( destination, __x+x, __y+y,  SDL_MapRGBA( destination->format, ((__id & 0xFF0000)>>16), ((__id & 0xFF00)>>8), (__id & 0xFF), 0x00 ) );
		}
}

void clean_surface( int __x, int __y, SDL_Surface* source, SDL_Surface* destination ) // last argument is optional (cf. function header)
{

	for ( int x = 0 ; x != source->w ; x++ )
		for ( int y = 0 ; y != source->h ; y++ )
		{
			Uint32 pixel = getPixel32(source,x,y);
			if ( pixel != SDL_MapRGBA( destination->format, 0xFF, 0xFF, 0xFF, 0 ) )
				putPixel32( destination, __x+x, __y+y, SDL_MapRGBA( destination->format, 0xFF, 0xFF, 0xFF, 0x00 ) );
		}
}

// used to register a robot into the environment (used since 2012-6-6)
void register_robotMask( int __x, int __y, SDL_Surface* destination , int __id ) // last argument is mandatory to track objects (in particular: agents)
{
    for ( int i = 0 ; (size_t)i != gRobotMaskData.size() ; i++ )
    {
        int x = __x + gRobotMaskData[i][0];
        int y = __y + gRobotMaskData[i][1];
        putPixel32( destination, x, y,  SDL_MapRGBA( destination->format, ( (__id & 0xFF0000)>>16), ((__id & 0xFF00)>>8), (__id & 0xFF), 0x00 ) );
    }
}

// used to unregister a robot from the environment (used since 2012-6-6)
void clean_robotMask( int __x, int __y, SDL_Surface* destination ) // last argument is optional (cf. function header)
{
    //std::cout << "gRobotMaskData.size() => " << gRobotMaskData.size() << std::endl;
    
    for ( unsigned int i = 0 ; i != gRobotMaskData.size() ; i++ )
    {
        int x = __x + gRobotMaskData[i][0];
        int y = __y + gRobotMaskData[i][1];
        
        //std::cout << "coordinates: " << x << " , " << y << std::endl;
        
        putPixel32( destination, x, y, SDL_MapRGBA( destination->format, 0xFF, 0xFF, 0xFF, 0x00 ) );
    }
}


/* **************** */
/* * SIMULATOR WINDOW * */
/* **************** */



void toggle_fullscreen()
{
	std::cout << "[critical] Fullscreen mode is not implemented." << std::endl;

	/*
	clean_up();

	gFullScreenMode= !gFullScreenMode;
	
	if( initSDL(SDL_HWSURFACE | SDL_DOUBLEBUF | (gFullScreenMode?SDL_FULLSCREEN:0) ) == false )
    {
		std::cerr << "fullscreen error 1 (toggling fullscreen) \n";
		exit(-2);
	}
    
    if( gWorld->loadFiles() == false ) // note: if this block is removed, computer may hang. + gWorld is not in the scope
    {
		std::cerr << "fullscreeen error 2 (re-loading files) \n";
		exit(-2);
	}
 
	//if (SDL_WM_ToggleFullScreen(gScreen) == 0)
	*/
}


SDL_Surface *load_image( std::string filename ) 
{
    //The image that's loaded
    SDL_Surface* loadedImage = NULL;
    
    //The optimized surface that will be used
    SDL_Surface* optimizedImage = NULL;
    
    //Load the image
    loadedImage = IMG_Load( filename.c_str() );
    
    //If the image loaded
    if( loadedImage != NULL )
    {
        //Create an optimized surface
        optimizedImage = SDL_DisplayFormat( loadedImage );
        
        //Free the old surface
        SDL_FreeSurface( loadedImage );
        
        //If the surface was optimized
        if( optimizedImage != NULL )
        {
            //Color key surface
            //SDL_SetColorKey( optimizedImage, SDL_SRCCOLORKEY, SDL_MapRGBA( optimizedImage->format, 0, 0xFF, 0xFF, 0 ) );
        }
    }
    
	//Return the optimized surface	
	return optimizedImage;
}


