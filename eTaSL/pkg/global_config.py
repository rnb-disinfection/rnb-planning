import os
import numpy as np

PROJ_DIR = os.getcwd()
TF_GMT_ETASL_DIR = os.environ['TF_GMT_ETASL_DIR']

OFFSET_DIR = "./offset/"
try: os.mkdir(OFFSET_DIR)
except: pass