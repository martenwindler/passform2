use std::net::{IpAddr, Ipv4Addr, UdpSocket, TcpStream};
use std::process::Command;
use std::time::Duration;

pub mod network {
    use super::*;

    const LOCALHOST: IpAddr = IpAddr::V4(Ipv4Addr::new(127, 0, 0, 1));

    /// Gibt die primäre lokale IP zurück (die IP, die "ins Internet" oder ins LAN zeigt)
    pub fn get_ip() -> IpAddr {
        UdpSocket::bind("0.0.0.0:0")
            .and_then(|socket| {
                socket.connect("8.8.8.8:80")?; // Nutzt Google DNS als Ziel, um Interface-Wahl zu erzwingen
                socket.local_addr()
            })
            .map(|addr| addr.ip())
            .unwrap_or(LOCALHOST)
    }

    /// Automatische Erkennung der Bay-ID ohne Schema-Vorgabe
    /// Sucht nach der ersten privaten IPv4 und extrahiert das dritte Oktett.
    pub fn auto_get_bay_id() -> Result<i32, String> {
        let ips = get_all_ips_internal();
        
        ips.iter()
            .find(|ip| is_private_ip(ip))
            .and_then(|ip| match ip {
                IpAddr::V4(v4) => Some(v4.octets()[2] as i32),
                _ => None,
            })
            .ok_or_else(|| format!("Keine private IP im LAN gefunden. Verfügbar: {:?}", ips))
    }

    /// Hilfsfunktion: Prüft ob eine IP im privaten Bereich liegt (192.168.x.x oder 10.x.x.x)
    fn is_private_ip(ip: &IpAddr) -> bool {
        match ip {
            IpAddr::V4(v4) => {
                let o = v4.octets();
                (o[0] == 192 && o[1] == 168) || (o[0] == 10) || (o[0] == 172 && (o[1] >= 16 && o[1] <= 31))
            }
            _ => false,
        }
    }

    /// Interne Hilfsfunktion für alle IPs
    fn get_all_ips_internal() -> Vec<IpAddr> {
        get_if_addrs::get_if_addrs()
            .map(|ifaces| ifaces.into_iter().filter(|i| !i.is_loopback()).map(|i| i.ip()).collect())
            .unwrap_or_else(|_| vec![LOCALHOST])
    }

    /// Prüft Erreichbarkeit (Ping)
    pub fn ip_reachable(host: &str) -> bool {
        Command::new("ping")
            .args(["-q", "-c", "1", "-W", "1", host])
            .status()
            .map(|s| s.success())
            .unwrap_or(false)
    }

    /// Prüft TCP Port
    pub fn is_port_in_use(host: &str, port: u16) -> bool {
        format!("{host}:{port}").parse()
            .map(|addr| TcpStream::connect_timeout(&addr, Duration::from_secs(1)).is_ok())
            .unwrap_or(false)
    }
}