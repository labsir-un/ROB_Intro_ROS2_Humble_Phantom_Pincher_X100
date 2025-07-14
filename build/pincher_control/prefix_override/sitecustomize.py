import sys
if sys.prefix == '/Users/amoralesma/conda/envs/humble':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/Users/amoralesma/prueba-ros/install/pincher_control'
