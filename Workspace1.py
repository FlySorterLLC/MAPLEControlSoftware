#! /usr/bin/env python

## Copyright (c) 2015, FlySorter, LLC

## Workspace configuration file

import numpy as np

import WorkspaceModules.FlyPlate
#import WorkspaceModules.Dispenser
#import WorkspaceModules.TriangleMaze
#import WorkspaceModules.CircleMaze

Workspace1 = { 'baseThickness': 2.93,
               'plate1': WorkspaceModules.FlyPlate.FlyPlate( np.array([50., 82.]),
                                                             np.array([138., 6.]) ) }
#               'dispenser1': WorkspaceModules.Dispenser.Dispenser( ( 600, 200) ),
#               'maze1': WorkspaceModules.TriangleMaze.TriangleMaze( (1, 3), (300, 300) ),
#               'maze2': WorkspaceModules.CircleMaze.CircleMaze( (501, 3), (800, 300) ) }
               
