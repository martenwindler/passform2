import yaml

def ros_name(uuid):
    return 'module_' + uuid.replace('-','_')

def from_ros_name(uuid):
    return uuid.replace('module_','').replace('_','-')

def load_yaml_file(yaml_file_path):
    try:
        with open(yaml_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError as e: # parent of IOError, OSError *and* WindowsError where available
        print(str(e))
        return None

if __name__ == '__main__':
    uuid = "129038123-12312-3123-123-123123"
    print(uuid)
    print(ros_name(uuid))
    print(from_ros_name(ros_name(uuid)))
