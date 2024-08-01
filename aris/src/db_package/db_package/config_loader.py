import os
import glob
import yaml
from ament_index_python.packages import get_package_share_directory

def load_config(config_file_name):
    try:
        package_share_directory = get_package_share_directory('db_package')
        
        config_path = os.path.join(package_share_directory, 'config')
        config_file_path = glob.glob(os.path.join(config_path, f"{config_file_name}.yaml"))

        if not config_file_path:
            raise FileNotFoundError(f"Configuration file '{config_file_name}.yaml' not found in '{config_path}'")

        with open(config_file_path[0], 'r') as file:
            try:
                config_data = yaml.safe_load(file)
                if config_data is None:
                    raise ValueError("Configuration file is empty or not properly formatted")
            except yaml.YAMLError as e:
                raise ValueError(f"Error parsing YAML file: {e}")

    except FileNotFoundError as e:
        print(f"FileNotFoundError: {e}")
        raise
    except ValueError as e:
        print(f"ValueError: {e}")
        raise
    except Exception as e:
        print(f"Unexpected error: {e}")
        raise

    return config_data

databaseConfig = load_config("database_config")
