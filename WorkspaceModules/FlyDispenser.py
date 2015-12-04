#! /usr/bin/env python

## Copyright (c) 2015, FlySorter, LLC

## Fly Dispenser class file

import numpy as np
import robotutil

class FlyDispenser:

    ZClearanceHeight = 50
    ZHeight = 45

    def __init__(self, dispenserPoint):

        self.dispenserPoint = dispenserPoint
        ## TODO: add serial class, instance of serial object
        ## and initialize dispenser (send "I")
        return

    # Returns 0 on success, 1 on failure
    def dispenseFly(self):
        # Send command ("F")

        # Get reply (to check whether fly was successfully dispensed or not)
        return 0
