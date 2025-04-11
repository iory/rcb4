import os
import re
import socket


def is_ros_master_local():
    ros_master_uri = os.environ.get('ROS_MASTER_URI', 'http://localhost:11311')
    match = re.match(r'http://([^:]+):\d+', ros_master_uri)
    if not match:
        return False

    master_host = match.group(1)

    if master_host in ['localhost', '127.0.0.1']:
        return True

    local_hostname = socket.gethostname()
    local_fqdn     = socket.getfqdn()
    local_ips      = socket.gethostbyname_ex(local_hostname)[2]

    if (master_host == local_hostname or
        master_host == local_fqdn or
        master_host in local_ips):
        return True

    return False

if __name__ == '__main__':
    if is_ros_master_local():
        print("ROS Master is running locally.")
    else:
        print("ROS Master is not running locally.")
