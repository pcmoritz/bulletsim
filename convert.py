#!/usr/bin/env python

import getopt
import sys
import os
from dolfin_utils.commands import getoutput
import re
import warnings
import os.path

import meshconvert
from dolfin_utils.meshconvert import xml_writer

def main(argv):
    ofilename = "/home/pcm/bulletsim/build/output.xml"
    handler = meshconvert.XmlHandler(ofilename)
    meshconvert.gmsh2xml("/home/pcm/bulletsim/build/output.msh",
                         handler)
    ofile = open(ofilename, "a")
    ofile.write("""\
  </mesh>
</dolfin>
""")
    
    

if __name__ == "__main__":
    main(sys.argv[1:])
