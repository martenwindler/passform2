#include "network.hpp"

namespace passform_driver{
namespace network{

std::vector<std::string> get_all_ip() {
    struct ifaddrs * ifAddrStruct=NULL;
    struct ifaddrs * ifa=NULL;
    void * tmpAddrPtr=NULL;

    getifaddrs(&ifAddrStruct);
    std::vector<std::string> addr_list; // list of all addresses

    for (ifa = ifAddrStruct; ifa != NULL; ifa = ifa->ifa_next) {
        if (ifa ->ifa_addr->sa_family==AF_INET) { // check it is IP4
            // is a valid IP4 Address
            tmpAddrPtr=&((struct sockaddr_in *)ifa->ifa_addr)->sin_addr;
            char addressBuffer[INET_ADDRSTRLEN];
            inet_ntop(AF_INET, tmpAddrPtr, addressBuffer, INET_ADDRSTRLEN);
            addr_list.push_back(std::string(addressBuffer));
            // printf("'%s': %s\n", ifa->ifa_name, addr_list.back().c_str());
         }
        //  else if (ifa->ifa_addr->sa_family==AF_INET6) { // check it is IP6
        //     // is a valid IP6 Address
        //     tmpAddrPtr=&((struct sockaddr_in6 *)ifa->ifa_addr)->sin6_addr;
        //     char addressBuffer[INET6_ADDRSTRLEN];
        //     inet_ntop(AF_INET6, tmpAddrPtr, addressBuffer, INET6_ADDRSTRLEN);
        //     addr_list.push_back(std::string(addressBuffer));
        //     // printf("'%s': %s\n", ifa->ifa_name, addr_list.back().c_str());
        // }
    }
    if (ifAddrStruct!=NULL)
        freeifaddrs(ifAddrStruct);//remember to free ifAddrStruct
    return addr_list;
}

std::vector<std::string> get__passform_ip(std::string scheme)
/*  Get all IP of the local machine matching "scheme" */
{
  auto addr_list = get_all_ip();
  std::vector<std::string> passform_list;

  for( auto addr : addr_list ) {
    if (addr.rfind(scheme, 0) == 0) { // pos=0 limits the search to the prefix
      passform_list.push_back(addr);
    }
  }
  return passform_list;
}


int get_bay_by_ip(std::vector<std::string>& addr_list, std::string scheme){
  int bay = -1;
  for( auto addr : addr_list ) {
    if (addr.rfind(scheme, 0) == 0) { // pos=0 limits the search to the prefix
      // printf("Potential address: %s\n", addr.c_str());
      bay = atoi(passform::util::split(addr)[2].c_str());
    }
  }
  if( bay == -1 ){
    throw std::runtime_error("No IP matching PassForM IP scheme ("+scheme+".<BAY>.x).");
  }
  // printf("It will be bay: %i\n", bay);
  return bay;
}


} // namespace network
} // namespace passform_module_comm
