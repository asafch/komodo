#!/usr/bin/env python

import argparse

def init_arguments( self ):
    parser = argparse.ArgumentParser()
    parser.add_argument('--is_simulation', action='store_true', dest="is_simulation", default=False )
    
    (options, args) = parser.parse_known_args()

    self.is_simulation = options.is_simulation

def pluto_add_namespace( is_simulation, topic ):
    if True == is_simulation:
        return topic
    else:
        return "/komodo_1/komodo_1" + topic
