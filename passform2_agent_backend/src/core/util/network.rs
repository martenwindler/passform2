use std::net::{IpAddr, Ipv4Addr, UdpSocket, TcpStream};
use std::process::Command;
use std::time::Duration;

pub mod network {
    use super::*;

    /// Gibt die primäre lokale IP-Adresse zurück (Dummy-UDP-Socket-Trick)
    pub fn get_ip() -> IpAddr {
        let socket = match UdpSocket::bind("0.0.0.0:0") {
            Ok(s) => s,
            Err(_) => return IpAddr::V4(Ipv4Addr::new(127, 0, 0, 1)),
        };

        // Muss nicht erreichbar sein, hilft dem OS aber, das richtige Interface zu wählen
        if socket.connect("10.255.255.255:1").is_err() {
            return IpAddr::V4(Ipv4Addr::new(127, 0, 0, 1));
        }

        match socket.local_addr() {
            Ok(addr) => addr.ip(),
            Err(_) => IpAddr::V4(Ipv4Addr::new(127, 0, 0, 1)),
        }
    }

    /// Gibt alle IPv4 Adressen der Netzwerk-Interfaces zurück
    /// Benötigt das Crate: get_if_addrs
    pub fn get_all_ip() -> Vec<IpAddr> {
        match get_if_addrs::get_if_addrs() {
            Ok(ifaces) => ifaces
                .into_iter()
                .filter(|iface| !iface.is_loopback())
                .map(|iface| iface.ip())
                .collect(),
            Err(_) => vec![IpAddr::V4(Ipv4Addr::new(127, 0, 0, 1))],
        }
    }

    /// Extrahiert die "Bay ID" basierend auf dem IP-Schema (z.B. 192.168.<BAY>.x)
    pub fn get_bay_by_ip(addr_list: &[IpAddr], scheme: &str) -> Result<i32, String> {
        for ip in addr_list {
            let ip_str = ip.to_string();
            if ip_str.starts_with(scheme) {
                let parts: Vec<&str> = ip_str.split('.').collect();
                if parts.len() >= 3 {
                    if let Ok(bay) = parts[2].parse::<i32>() {
                        return Ok(bay);
                    }
                }
            }
        }
        Err(format!(
            "No IP matching PassForM IP scheme ({}.<BAY>.x). Active IPs: {:?}",
            scheme, addr_list
        ))
    }

    /// Prüft die Erreichbarkeit eines Hosts via System-Ping
    pub fn ip_reachable(host: &str) -> bool {
        let status = Command::new("ping")
            .arg("-q")
            .arg("-c")
            .arg("2")
            .arg("-W")
            .arg("1")
            .arg(host)
            .status();

        match status {
            Ok(s) => s.success(),
            Err(_) => false,
        }
    }

    /// Prüft, ob ein TCP-Port auf einem Host belegt ist
    pub fn is_port_in_use(host: &str, port: u16) -> bool {
        let address = format!("{}:{}", host, port);
        // Timeout ist wichtig, damit Rust nicht ewig wartet
        TcpStream::connect_timeout(
            &address.parse().expect("Invalid address"),
            Duration::from_secs(1)
        ).is_ok()
    }
}