import rospy

from kxr_controller.module_loader import ModuleLoader

_client_loader = ModuleLoader('dynamic_reconfigure.client', 'Client')

def update_kxr_parameters(server_name='rcb4_ros_bridge', frame_count=None,
                          wheel_frame_count=None, temperature_limit=None,
                          current_limit=None,
                          namespace=None,
                          **kwargs):
    if namespace is not None:
        server_name = namespace + '/' + server_name
    Client = _client_loader.get_module()
    client = Client(server_name, timeout=5.0)
    cfg = {}
    if frame_count is not None:
        cfg['frame_count'] = frame_count
    if wheel_frame_count is not None:
        cfg['wheel_frame_count'] = wheel_frame_count
    if temperature_limit is not None:
        cfg['temperature_limit']  = temperature_limit
    if current_limit is not None:
        cfg['current_limit'] = current_limit

    updated = client.update_configuration(cfg)
    rospy.loginfo(f"[update_kxr_parameters] Change reconfigure parameters: {updated}")
    return updated
