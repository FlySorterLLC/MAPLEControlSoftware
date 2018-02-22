##
## This copyrighted software is distributed under the GPL v2.0 license.
## See the LICENSE file for more details.
##

## Workspace configuration file

import numpy as np

import WorkspaceModules.YeastApplicatorPlate
import WorkspaceModules.YeastArena
import WorkspaceModules.YeastArena3x3

YeastWorkspace = { 'baseThickness': 2.93, 'yeastApplicatorPlate': WorkspaceModules.YeastApplicatorPlate.YeastApplicatorPlate(422.0, 247), 
					'yeastArena': WorkspaceModules.YeastArena.YeastArena(285, 139),
					'yeastArena3x3': WorkspaceModules.YeastArena3x3.YeastArena3x3(124, 36) }
