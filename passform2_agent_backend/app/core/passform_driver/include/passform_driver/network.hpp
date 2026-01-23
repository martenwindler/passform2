#ifndef PASSFORM_DRIVER__NETWORK_HPP
#define PASSFORM_DRIVER__NETWORK_HPP

#include <stdio.h>
#include <sys/types.h>
#include <ifaddrs.h>
#include <netinet/in.h>
#include <string.h>
#include <arpa/inet.h>
#include <vector>
#include <string>
#include <stdexcept>

#include "passform_util/helper.hpp"

namespace passform_driver{
namespace network{

std::vector<std::string> get_all_ip();
std::vector<std::string> get__passform_ip(std::string scheme = "192.168.");
int get_bay_by_ip(std::vector<std::string>& addr_list, std::string scheme = "192.168.");

} // ns network
} // ns passform_driver
#endif
