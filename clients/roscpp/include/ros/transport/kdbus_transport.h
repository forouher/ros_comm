#include <string>
#include <cstdlib>

#include <stdio.h>
#include <string.h>
#include <time.h>
#include <fcntl.h>
#include <stdlib.h>
#include <stddef.h>
#include <unistd.h>
#include <stdint.h>
#include <errno.h>
#include <assert.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include "ros/memfd_message.h"

#include <boost/interprocess/managed_external_buffer.hpp>

extern "C"
{

#include "kdbus-util.h"
#include "kdbus-enum.h"
}

namespace ros {

class KDBusTransport {

public:

    KDBusTransport(const std::string& name);

    int create_bus();

    int destroy_bus();

    int open_connection(const std::string& name);

    int close_connection();

    bool sendMessage(MemfdMessage::Ptr msg, const std::string& receiver);

    MemfdMessage::Ptr receiveMessage();

    // connection
    struct conn *conn;

private:

    // bus
    std::string bus;
    std::string buspath;
    int fdc;

};

}

