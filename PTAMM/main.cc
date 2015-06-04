// Copyright 2009 Isis Innovation Limited
// This is the main extry point for PTAMM
#include <stdlib.h>
#include <iostream>
#include <gvars3/instances.h>
#include "System.h"

#include "ghostsettings.h"

using namespace std;
using namespace GVars3;
using namespace PTAMM;

int main()
{
  cout << "  Welcome to PTAMM " << endl;
  cout << "  ---------------- " << endl;
  cout << "  Parallel tracking and multiple mapping" << endl;
  cout << "  Copyright (C) Isis Innovation Limited 2009 " << endl;
  cout << endl;
  cout << "  Parsing settings.cfg ...." << endl;

#if GHOST_INPUT == INPUT_KINECT
  GUI.LoadFile("settingskinect.cfg");
#endif
#if GHOST_INPUT == INPUT_OPENNI
  GUI.LoadFile("settingsopenni.cfg");
#endif
#if GHOST_INPUT == INPUT_VI
  GUI.LoadFile("settings_vi.cfg");
#endif
#if GHOST_INPUT == INPUT_KINECT2
  GUI.LoadFile("settings_kinect2.cfg");
#endif
#if GHOST_INPUT == INPUT_WEBCAM
  GUI.LoadFile("settings.cfg");
#endif

  GUI.StartParserThread(); // Start parsing of the console input
  atexit(GUI.StopParserThread); 
  
  try
    {
      System s;
      s.Run();
    }
  catch(CVD::Exceptions::All e)
    {
      cout << endl;
      cout << "!! Failed to run system; got exception. " << endl;
      cout << "   Exception was: " << endl;
      cout << e.what << endl;
    }

  return 0;
}



