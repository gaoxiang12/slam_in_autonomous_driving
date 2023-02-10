#include "socket_input.h"

namespace driver {
namespace velodyne {

////////////////////////////////////////////////////////////////////////
// InputSocket class implementation
////////////////////////////////////////////////////////////////////////

/** @brief constructor
 *
 *  @param private_nh private node handle for driver
 *  @param udp_port UDP port number to connect
 */
SocketInput::SocketInput(std::string device_ip_str, uint16_t port)
    : sockfd_(-1), ip_str_(device_ip_str), port_(port) {}

/** @brief destructor */
SocketInput::~SocketInput(void) {
  MY_LOG_INFO("------ socket closed --------");
  (void)close(sockfd_);
}

int SocketInput::Init() {
  // initialize device ip
  memset(&devip_, 0, sizeof(in_addr));
  memset(&my_addr_, 0, sizeof(my_addr_));        // initialize to zeros
  my_addr_.sin_family = AF_INET;                 // host byte order
  my_addr_.sin_port = htons(port_);              // port in network byte order
  my_addr_.sin_addr.s_addr = htonl(INADDR_ANY);  // automatically fill in my IP
  if (sockfd_ != -1) {
    (void)close(sockfd_);
  }

  // check ip str
  if (ip_str_.empty()) {
    MY_LOG_ERROR("create ip address str is empty");
    return static_cast<int>(Socket::SocketErrorCode::IP_EMPTY);
  }
  // swich ip str to in_addr
  if (inet_aton(ip_str_.c_str(), &devip_) == 0) {
    MY_LOG_ERROR("ip address is incorrect");
    return static_cast<int>(Socket::SocketErrorCode::IP_WRONG);
  }
  // create socket
  // AF_INET:IPV4 SOCK_DGRAM:UDP
  if ((sockfd_ = socket(AF_INET, SOCK_DGRAM, 0)) == -1) {
    MY_LOG_ERROR("init socket failed");
    return static_cast<int>(Socket::SocketErrorCode::SOCKET_CREATE_FAILED);
  }

  int val = 1;
  if (-1 == setsockopt(sockfd_, SOL_SOCKET, /*OS_DONTLINGER*/ SO_REUSEADDR,
                       (void *)&val, sizeof(int))) {
    MY_LOG_ERROR("bind socket failed");
    return static_cast<int>(Socket::SocketErrorCode::SOCKET_OPTION_FAILED);
  }

  // bind socket with port
  if (bind(sockfd_, (sockaddr *)&my_addr_, sizeof(sockaddr)) < 0) {
    MY_LOG_ERROR("bind socket failed");
    return static_cast<int>(Socket::SocketErrorCode::SOCKET_BIND_FAILED);
  }

  if (fcntl(sockfd_, F_SETFL, O_NONBLOCK | FASYNC) < 0) {
    // TODO: error code
    MY_LOG_ERROR("set socket to non-block failed");
    return static_cast<int>(Socket::SocketErrorCode::SOCKET_NON_BLOCK_FAILED);
  }

  MY_LOG_INFO("Connect socket fd is " << sockfd_);
  return static_cast<int>(Socket::SocketErrorCode::OK);
}

/** @brief Get one velodyne packet. */
int SocketInput::ReceivePacket(void *data, const int length, uint64_t *nsec) {
  sockaddr_in sender_address;
  socklen_t sender_address_len = sizeof(sender_address);

  while (true) {
    if (!input_available(kSocketPollTimeout)) {
      // FIXME: TBD if it is nessisery to return
      return static_cast<int>(Socket::SocketErrorCode::SOCKET_POLL_TIMEOUT);
    }
    // Receive packets that should now be available from the
    // socket using a blocking read.
    std::vector<uint8_t> fake_data = std::vector<uint8_t>(1206);
    ssize_t nbytes =
        recvfrom(sockfd_, /*&fake_data[0]*/ data, static_cast<size_t>(length),
                 0, (sockaddr *)&sender_address, &sender_address_len);

    if (nbytes < 0) {
      // TODO: error code
      MY_LOG_ERROR("recvfail from port " << port_);
      return static_cast<int>(Socket::SocketErrorCode::SOCKET_RECV_FAILED);
    } else if (nbytes == length) {
      // read successful,
      // if packet is not from the lidar scanner we selected by IP,
      // continue otherwise we are done
      if (sender_address.sin_addr.s_addr != devip_.s_addr) {
        ROS_WARN(
            "desired ip is: %s, but recieve data from other ip: %s, "
            "multi-lidar port conflict !!\n",
            inet_ntoa(devip_), inet_ntoa(sender_address.sin_addr));
        return static_cast<int>(
            Socket::SocketErrorCode::SOCKET_DATA_UNMATCHED_IP);
      } else {
        // ROS_DEBUG_STREAM("complete packet read: " << nbytes << "bytes");
        break;
      }
    }
  }

  *nsec = ros::Time::now().toNSec();
  return static_cast<int>(Socket::SocketErrorCode::OK);
}

bool SocketInput::input_available(int timeout) {
  struct pollfd fds[1];
  fds[0].fd = sockfd_;
  fds[0].events = POLLIN;
  // Unfortunately, the Linux kernel recvfrom() implementation
  // uses a non-interruptible sleep() when waiting for data,
  // which would cause this method to hang if the device is not
  // providing data.  We poll() the device first to make sure
  // the recvfrom() will not block.
  //
  // Note, however, that there is a known Linux kernel bug:
  //
  //   Under Linux, select() may report a socket file descriptor
  //   as "ready for reading", while nevertheless a subsequent
  //   read blocks.  This could for example happen when data has
  //   arrived but upon examination has wrong checksum and is
  //   discarded.  There may be other circumstances in which a
  //   file descriptor is spuriously reported as ready.  Thus it
  //   may be safer to use O_NONBLOCK on sockets that should not
  //   block.

  // poll() until input available
  do {
    int retval = poll(fds, 1, timeout);

    if (retval < 0) {  // poll() error?
      // TODO: error code
      // EBADF | EFAULTfds | EINTR | EINVALnfds | ENOMEM
      // MY_LOG_ERROR_ONCE("poll error");
      return false;
    } else if (retval == 0) {  // poll() timeout?
      // TODO: error code
      // ROS_WARN_STREAM("Velodyne port " << port_ << " poll() timeout");
      return false;
    }

    if ((fds[0].revents & POLLERR) || (fds[0].revents & POLLHUP) ||
        (fds[0].revents & POLLNVAL)) {  // device error?
      // TODO: error code
      // MY_LOG_ERROR("Velodyne port " << port_
      //                                   << "poll() reports Velodyne error");
      return false;
    }
  } while ((fds[0].revents & POLLIN) == 0);
  return true;
}

}  // namespace velodyne
}  // namespace driver
