#! /usr/bin/env python

## Copyright (c) 2015, FlySorter, LLC

## Workspace configuration file

import numpy as np

import WorkspaceModules.FlyPlate
import WorkspaceModules.FlyDispenser

Workspace1 = { 'baseThickness': 2.93,
               'plate1': WorkspaceModules.FlyPlate.FlyPlate( np.array([43.6, 91.1]),
                                                             np.array([142.3, 27.4]) ),
               'dispenser1': WorkspaceModules.FlyDispenser.FlyDispenser( ( 186.0, 54.2) ) }
               
