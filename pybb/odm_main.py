#!/usr/bin/env python3
import sys
from black_box.odm.odm import BlackBoxODM

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print('Usage: python odm.py [db_name]')
    db_name = sys.argv[1]
    bb_odm = BlackBoxODM(db_name)
    bb_odm.deserialize_logs()
