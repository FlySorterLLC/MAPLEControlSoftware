#! /usr/bin/env python

## Copyright (c) 2015, FlySorter, LLC

## Workspace configuration file

import numpy as np

import WorkspaceModules.FlyPlate
#import WorkspaceModules.Dispenser
#import WorkspaceModules.TriangleMaze
#import WorkspaceModules.CircleMaze

Workspace1 = { 'plate1': WorkspaceModules.FlyPlate.FlyPlate( np.array([205.,70.3]),
                                                             np.array((102.,129.3)) ) }
#               'dispenser1': WorkspaceModules.Dispenser.Dispenser( ( 600, 200) ),
#               'maze1': WorkspaceModules.TriangleMaze.TriangleMaze( (1, 3), (300, 300) ),
#               'maze2': WorkspaceModules.CircleMaze.CircleMaze( (501, 3), (800, 300) ) }
               
