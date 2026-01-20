#!/usr/bin/env python3
import re
import urllib

def sanitize_basyx_string(s):
    """remove all chars not allowed in id_short and replace with underscore"""
    return re.subn('[ `~!@#$%^&*()-+={[}}|\:;<,>.?/-]', '_', s.lower())[0]

def sanitize_id(aas_id) -> str:
    """Replaces unsafe characters from AAS id using the %xx escape (e.g. '/' -> '%2F')"""
    return urllib.parse.quote(aas_id, safe='')

def response_to_exception(r):
    try:
        msg = r.json()['messages'][0]
        return f'Code {msg["code"]}: {msg["text"]}'
    except Exception:
        return f'{r}: Empty message'

def split_host(host:str, port:int=None):
    head, sep, tail = host.partition(':')
    if len(tail)>0:
        port_new = int(tail)
        if port is not None:
            assert port_new == port, f'Host port {host} and specified port {port} do not match.'
    else:
        port_new = port
    assert port_new is not None, 'Port missing.'
    return head, port_new
