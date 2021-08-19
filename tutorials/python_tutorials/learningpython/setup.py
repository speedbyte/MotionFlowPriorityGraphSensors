""" 


    $Date$
    $Rev$
    $Author$
    $URL$

    Copyright (c) 2007 S1nn GmbH & Co. KG.
    All Rights Reserved.

"""

_svn_id = "$Id$"
_svn_rev = "$Rev$"
__version__ = _svn_rev[6:-2]

from distutils.core import setup
import py2exe


setup(console=['DiagApp_GUI.py'])