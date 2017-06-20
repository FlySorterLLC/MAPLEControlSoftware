#! /usr/bin/env python

## Copyright (c) 2015, FlySorter, LLC

## Workspace configuration file

import numpy as np

import WorkspaceModules.TriangleMaze
import WorkspaceModules.FlyPad

Workspace2 = { 'baseThickness': 1.58,
               'maze1': WorkspaceModules.TriangleMaze.TriangleMaze( np.array([92., 124.]),
                                                                    np.array([201.7, 17.]) ),
               'pad1': WorkspaceModules.FlyPad.FlyPad( np.array([283., 45.]),
                                                       np.array([313., 25.]) ) }
#               'pad1': WorkspaceModules.FlyPad.FlyPad( np.array([283., 126.]),
#                                                       np.array([363., 25.]) ) }
               
