#!/usr/bin/env python
import sys
import time
import yaml
from black_box.query_interface.query_interface import BlackBoxQueryInterface

class ConfigParams(object):
    def __init__(self):
        self.bb_id = ''
        self.zyre_groups = list()
        self.data_sources = list()

def get_config_params(config_file):
    config_data = dict()
    with open(config_file, 'r') as bb_config:
        config_data = yaml.load(bb_config)

    config_params = ConfigParams()
    for data_item in config_data:
        # each list item is a dictionary with a single key-value pair,
        # where the key is the data source and the value is a set of
        # configuration parameters for that source
        data_source, data = next(iter(data_item.items()))
        if data_source == 'default_parameters':
            continue

        if data_source == 'zyre':
            config_params.bb_id = data['name']
            config_params.zyre_groups = data['groups']
        config_params.data_sources.append(data_source)
    return config_params

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print('Usage: query_interface_main.py <absolute-path-to-black-box-config-file>')
    bb_config_file = sys.argv[1]
    config_params = get_config_params(bb_config_file)
    query_interface = BlackBoxQueryInterface(config_params.data_sources,
                                             config_params.bb_id,
                                             config_params.zyre_groups)

    print('[{0}] Query interface initialised'.format(config_params.bb_id))
    try:
        while True:
            time.sleep(0.5)
    except (KeyboardInterrupt, SystemExit):
        query_interface.shutdown()
        print('[{0}] Query interface interrupted; exiting'.format(config_params.bb_id))
