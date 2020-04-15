# import sys so we can use packages outside of this folder in
# either python 2 or python 3
import sys
import os
from pathlib import Path 
#insert parent directory into the path
sys.path.insert(0,str(Path(os.path.abspath(__file__)).parent.parent))

from input_output.hdf5datasetwriter import HDF5DatasetWriter

""" This class will create an hdf5 file. This will allow one to train models on data that is 
too large to be held in main memory at one time"""

