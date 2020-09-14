#!/usr/bin/env python3
import os, sys, time, random, pickle

import numpy as np
import pandas as pd

from IPython import embed

current_file_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.join(current_file_path, '..', '..'))

from scripts.utils import build_gam

DOMAIN_PATH = os.path.join(current_file_path, '..', '..', '..', 'domains', 'CDB')
FEEDBACK_PATH = os.path.join(DOMAIN_PATH, 'feedback')
PARAM_PATH = os.path.join(DOMAIN_PATH, 'params')


# Build and save the gam and gam_map for the action 'open'
print("Trying to open feedback data ")
print(pd.read_csv(os.path.join(FEEDBACK_PATH, 'open.data')))
open_gam, open_gam_map = build_gam(pd.read_csv(os.path.join(FEEDBACK_PATH, 'open.data')))
print(open_gam)