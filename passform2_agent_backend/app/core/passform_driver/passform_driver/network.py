import socket
import netifaces

def get_ip():
    """
    method returns the "primary" IP on the local box (the one with a default route)
    https://stackoverflow.com/a/28950776
    """
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.settimeout(0)
    try:
        # doesn't even have to be reachable
        s.connect(('10.255.255.255', 1))
        IP = s.getsockname()[0]
    except Exception:
        IP = '127.0.0.1'
    finally:
        s.close()
    return IP

def get_all_ip():
    """ gets all addresses of the netifaces """
    ip = list()
    for iface in netifaces.interfaces():
        iface_details = netifaces.ifaddresses(iface)
        if netifaces.AF_INET in iface_details:
            for ip_interfaces in iface_details[netifaces.AF_INET]:
                ip.append(ip_interfaces['addr'])
    return ip

def get_bay_by_ip(addr_list, scheme = '192.168'):
    """ """
    if type(addr_list) != list:
        addr_list = [addr_list]
    for ip in addr_list:
        if ip[:len(scheme)] == scheme:
            return int( ip.split('.')[2] )
    raise RuntimeError('No IP matching PassForM IP scheme ({}.<BAY>.x). Active IPs {}'.format(scheme, addr_list))




if __name__ == '__main__':
    bay_id = get_bay_by_ip( ['::1','192.168.178.202'] )
    print('Module is currently active in bay {}'.format(bay_id))
    # import nmap3
    # nmap = nmap3.NmapScanTechniques()
    # result = nmap.nmap_ping_scan("192.168.0.134")
    # print(result)
