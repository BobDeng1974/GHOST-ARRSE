// -*- c++ -*-
// Copyright 2009 Isis Innovation Limited

/***********************************************************************

  Use this file to place the headers of your games.
  This way only one header file needs to be included

  Also add your games to the functions below

*************************************************************************/

#ifndef _GAMES_H_
#define _GAMES_H_

#include "Game.h"
#include "EyeGame.h"

#if ENABLE_MODELS_GAME
#include "ModelsGame.h"
#endif
#include "ShooterGame.h"

///@TODO Place the header files for your games here.
//#include "GhostGame.h"
#include "ARReenactmentGame.h"
#include "ARCaptureGame.h"

namespace PTAMM {

Game * LoadAGame( std::string sName, std::string sGameDataFileName );
void InitializeGameMenu();
}

#endif
