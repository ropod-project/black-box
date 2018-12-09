import black_box.config.config_params as config_params
import yaml

class ConfigFileReader(object):
    @staticmethod
    def load_config(config_file_name):
        params = config_params.ConfigParams()

        config_data = dict()
        try:
            with open(config_file_name, 'r') as config_file:
                config_data = yaml.load(config_file)
        except Exception as exc:
            print('[config_file_reader] An error occured while reading {0}'.format(config_file_name))
            print(exc)
            return config_data

        for param_group in config_data:
            key = next(iter(param_group))
            config_data = param_group[key]
            if key == config_params.ConfigKeys.DEFAULT_PARAMETERS:
                if 'max_frequency' in config_data:
                    params.default.max_frequency = config_data['max_frequency']
                else:
                    raise Exception('default_parameters: max_frequency not specified')

                if 'max_database_size' in config_data:
                    params.default.max_db_size = config_data['max_database_size']
                else:
                    raise Exception('default_parameters: max_database_size not specified')

                if 'split_database' in config_data:
                    params.default.split_db = config_data['split_database']
                else:
                    print('default_parameters: split_database not specified; using default False')
            elif key == config_params.ConfigKeys.ROS:
                if 'ros_master_uri' in config_data:
                    params.ros.ros_master_uri = config_data['ros_master_uri']
                else:
                    raise Exception('ros: ros_master_uri not specified')

                if 'topics' in config_data:
                    for topic_data_group in config_data['topics']:
                        topic_data = topic_data_group[next(iter(topic_data_group))]
                        topic_params = config_params.RosTopicParams()

                        if 'name' in topic_data:
                            topic_params.name = topic_data['name']
                        else:
                            raise Exception('ros: ROS topic name not specified')

                        if 'type' in topic_data:
                            slash_idx = topic_data['type'].rfind('/')
                            msg_pkg = '{0}.msg'.format(topic_data['type'][0:slash_idx])
                            msg_type = topic_data['type'][slash_idx+1:]
                            topic_params.msg_pkg = msg_pkg
                            topic_params.msg_type = msg_type
                        else:
                            raise Exception('ros: type not specified for {0}'.format(topic_data['name']))

                        if 'max_frequency' in topic_data:
                            topic_params.max_frequency = topic_data['max_frequency']
                        else:
                            print('ros: max_frequency not specified; using default {0}'.format(params.default.max_frequency))
                            topic_params.max_frequency = params.default.max_frequency

                        params.ros.topic.append(topic_params)
                else:
                    raise Exception('ros: topics not specified')
            elif key == config_params.ConfigKeys.ZYRE:
                if 'name' in config_data:
                    params.zyre.node_name = config_data['name']
                else:
                    raise Exception('zyre: node_name not specified')

                if 'groups' in config_data:
                    params.zyre.groups = config_data['groups']
                else:
                    raise Exception('zyre: groups not specified')

                if 'message_types' in config_data:
                    params.zyre.message_types = config_data['message_types']
                else:
                    raise Exception('zyre: message_types not specified')

        return params
