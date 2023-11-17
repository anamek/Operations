import numpy as np
import pandas as pd
import os
import random
from itertools import product
from gurobipy import Model,GRB,LinExpr,quicksum
from operator import itemgetter
import matplotlib.pyplot as plt