// -*- c++ -*-
// Copyright 2009 Isis Innovation Limited
//
//
//  Add your games to the functions below
//
//

#include "Games.h"
#include <gvars3/instances.h>

namespace PTAMM {

using namespace GVars3;
  
/**
 * This function is called by the MapSerializer to load a game based on the name found in a map file.
 * It is also caller by the ARDriver to load a game when the user selects it from the GUI menu.
 * The path is only used when loading from disk and is the path to the saved data file.
 * @param sName Game name
 * @param sGameDataFileName saved game data file name
 * @return pointer to the loaded game
 */
Game * LoadAGame( std::string sName, std::string sGameDataFileName)
{
  Game * pGame = NULL;
  
  if( sName == "None" )
  {
    cout << "No game to load." << endl;
    return NULL;
  }

  if( sName == "Eyes" )  {
    pGame = new EyeGame();
  }
  else if( sName == "Shooter" )  {
    pGame = new ShooterGame();
  }
#ifdef ENABLE_MODELS_GAME
  else if( sName == "Models" )  {
    pGame = new ModelsGame();
  }
#endif
  ///@TODO Add your games here.
//  else if( sName == "MY_AR_GAME" )  {
//    pGame = new MyARGame();
//  }
  else if ( sName == "Ghost" ) {
	  //pGame = new GHOST::GhostGame();
  }
  else if (sName == "AR Reenactment"){
	  pGame = new ARReenactmentGame();
  }
  else if (sName == "AR Capture"){
	  pGame = new ARCaptureGame();
  }
  else
  {
    cout << "ERROR Unknown game: " << sName << endl;
    return NULL;
  }

  // Load the game data
  if( !sGameDataFileName.empty() ) {
    pGame->Load(sGameDataFileName);
  }

  return pGame;
}


/**
 * Create the game menu buttons for each game
 * Add your games to this list so that they appear in the menu
 */
void InitializeGameMenu()
{
  GUI.ParseLine("Menu.AddMenuButton Demos None \"LoadGame None\" Root");
  //GUI.ParseLine("Menu.AddMenuButton Demos Eyes \"LoadGame Eyes\" Root");
  //GUI.ParseLine("Menu.AddMenuButton Demos Shooter \"LoadGame Shooter\" Root");
  //GUI.ParseLine("Menu.AddMenuButton Demos Models \"LoadGame Models\" Root");
  //GUI.ParseLine("Menu.AddMenuButton Demos GHOST \"LoadGame Ghost\" Root");
  GUI.ParseLine("Menu.AddMenuButton Demos Reenact \"LoadGame AR Reenactment\" Root");
  GUI.ParseLine("Menu.AddMenuButton Demos Calibrate \"LoadGame AR Capture\" Root");

  ///@TODO Add you games here using this template:
  // GUI.ParseLine("Menu.AddMenuButton Demos BUTTONLABEL \"LoadGame MY_AR_GAME\" Root");
  // change BUTTONLABEL to the text you want
  // change MY_AR_GAME to the name used above.
}

}
