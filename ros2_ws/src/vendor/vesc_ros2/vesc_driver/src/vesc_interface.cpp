// Copyright 2020 F1TENTH Foundation
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//   * Redistributions of source code must retain the above copyright
//     notice, this list of conditions and the following disclaimer.
//
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//
//   * Neither the name of the {copyright_holder} nor the names of its
//     contributors may be used to endorse or promote products derived from
//     this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

// -*- mode:c++; fill-column: 100; -*-

#include "vesc_driver/vesc_interface.hpp"

#include <algorithm>
#include <atomic>
#include <cassert>
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <unistd.h>
#include <vector>

#include "vesc_driver/vesc_packet_factory.hpp"
#include "serial_driver/serial_driver.hpp"

namespace vesc_driver
{

class VescInterface::Impl
{
public:
  Impl()
  : packet_thread_run_(false),
    owned_ctx{new IoContext(2)},
    serial_driver_{new drivers::serial_driver::SerialDriver(*owned_ctx)},
    legacy_protocol_(false),
    read_fd_(-1)
  {}
  void packet_creation_thread();
  void on_configure();
  void connect(const std::string & port);
  void reset_buffer() {buffer_.clear();}

  std::atomic<bool> packet_thread_run_;
  std::unique_ptr<std::thread> packet_thread_;
  PacketHandlerFunction packet_handler_;
  ErrorHandlerFunction error_handler_;
  std::unique_ptr<drivers::serial_driver::SerialPortConfig> device_config_;
  std::string device_name_;
  std::unique_ptr<IoContext> owned_ctx{};
  std::unique_ptr<drivers::serial_driver::SerialDriver> serial_driver_;
  std::atomic<bool> legacy_protocol_;
  int read_fd_;

  ~Impl()
  {
    if (owned_ctx) {
      owned_ctx->waitForExit();
    }
  }

private:
  std::vector<uint8_t> buffer_;
};

void VescInterface::Impl::packet_creation_thread()
{
  auto temp_buffer = Buffer(2048, 0);
  while (packet_thread_run_.load()) {
    const ssize_t result = ::read(read_fd_, temp_buffer.data(), temp_buffer.size());
    if (result < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
        continue;
      }
      if (packet_thread_run_.load() && error_handler_) {
        error_handler_(
          std::string("Serial receive failed: ") + std::strerror(errno));
      }
      break;
    }
    if (result == 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
      continue;
    }
    const size_t bytes_read = static_cast<size_t>(result);
    if (!packet_thread_run_.load()) {
      break;
    }
    buffer_.reserve(buffer_.size() + temp_buffer.size());
    buffer_.insert(buffer_.end(), temp_buffer.begin(), temp_buffer.begin() + bytes_read);
    int bytes_needed = VescFrame::VESC_MIN_FRAME_SIZE;
    if (!buffer_.empty()) {
      // search buffer for valid packet(s)
      auto iter = buffer_.begin();
      auto iter_begin = buffer_.begin();
      while (iter != buffer_.end()) {
        // check if valid start-of-frame character
        if (VescFrame::VESC_SOF_VAL_SMALL_FRAME == *iter ||
          VescFrame::VESC_SOF_VAL_LARGE_FRAME == *iter)
        {
          // good start, now attempt to create packet
          std::string error;
          VescPacketConstPtr packet =
            VescPacketFactory::createPacket(iter, buffer_.end(), &bytes_needed, &error);
          if (packet) {
            // good packet, check if we skipped any data
            if (std::distance(iter_begin, iter) > 0) {
              std::ostringstream ss;
              ss << "Out-of-sync with VESC, unknown data leading valid frame. Discarding " <<
                std::distance(iter_begin, iter) << " bytes.";
              error_handler_(ss.str());
            }
            // call packet handler
            packet_handler_(packet);
            // update state
            iter = iter + packet->frame().size();
            iter_begin = iter;
            // continue to look for another frame in buffer
            continue;
          } else if (bytes_needed > 0) {
            // need more data, break out of while loop
            break;  // for (iter_sof...
          } else {
            // else, this was not a packet, move on to next byte
            error_handler_(error);
          }
        }

        iter++;
      }

      // if iter is at the end of the buffer, more bytes are needed
      if (iter == buffer_.end()) {
        bytes_needed = VescFrame::VESC_MIN_FRAME_SIZE;
      }

      // erase "used" buffer
      if (std::distance(iter_begin, iter) > 0) {
        std::ostringstream ss;
        ss << "Out-of-sync with VESC, discarding " << std::distance(iter_begin, iter) << " bytes.";
        error_handler_(ss.str());
      }
      buffer_.erase(buffer_.begin(), iter);
    }
    // Only attempt to read every 5 ms
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
}

void VescInterface::Impl::connect(const std::string & port)
{
  uint32_t baud_rate = 115200;
  // The vehicle VESC UART harness only carries TX, RX, and ground. Enabling
  // RTS/CTS makes CH340 adapters issue modem-control transfers that can hang
  // the shared USB-UART hub even though firmware discovery initially works.
  auto fc = drivers::serial_driver::FlowControl::NONE;
  auto pt = drivers::serial_driver::Parity::NONE;
  auto sb = drivers::serial_driver::StopBits::ONE;
  device_config_ =
    std::make_unique<drivers::serial_driver::SerialPortConfig>(baud_rate, fc, pt, sb);
  serial_driver_ = std::make_unique<drivers::serial_driver::SerialDriver>(*owned_ctx);
  serial_driver_->init_port(port, *device_config_);
  if (!serial_driver_->port()->is_open()) {
    serial_driver_->port()->open();
  }
  // serial_driver::receive() is a blocking read which cannot reliably be
  // interrupted by close() from another thread on Linux TTYs.  Read through
  // a second, non-blocking descriptor instead; the SerialDriver descriptor is
  // retained for asynchronous writes and port configuration.
  read_fd_ = ::open(port.c_str(), O_RDONLY | O_NOCTTY | O_NONBLOCK);
  if (read_fd_ < 0) {
    const std::string reason = std::strerror(errno);
    serial_driver_->port()->close();
    throw std::runtime_error(
            "Failed to open non-blocking VESC reader: " + reason);
  }
}

VescInterface::VescInterface(
  const std::string & port,
  const PacketHandlerFunction & packet_handler,
  const ErrorHandlerFunction & error_handler)
: impl_(new Impl())
{
  setPacketHandler(packet_handler);
  setErrorHandler(error_handler);
  // attempt to conect if the port is specified
  if (!port.empty()) {
    connect(port);
  }
}

VescInterface::~VescInterface()
{
  disconnect();
}

void VescInterface::setPacketHandler(const PacketHandlerFunction & handler)
{
  // todo - definately need mutex
  impl_->packet_handler_ = handler;
}

void VescInterface::setErrorHandler(const ErrorHandlerFunction & handler)
{
  // todo - definately need mutex
  impl_->error_handler_ = handler;
}

void VescInterface::connect(const std::string & port)
{
  // todo - mutex?

  if (isConnected()) {
    throw SerialException("Already connected to serial port.");
  }

  impl_->reset_buffer();

  // connect to serial port
  try {
    impl_->connect(port);
  } catch (const std::exception & e) {
    std::stringstream ss;
    ss << "Failed to open the serial port " << port << " to the VESC. " << e.what();
    throw SerialException(ss.str().c_str());
  }

  // start up a monitoring thread
  impl_->packet_thread_run_ = true;
  impl_->packet_thread_ = std::unique_ptr<std::thread>(
    new std::thread(
      &VescInterface::Impl::packet_creation_thread, impl_.get()));
}

void VescInterface::disconnect()
{
  // todo - mutex?

  // Closing the descriptor must happen before join().  receive() is a
  // blocking call; asking the VESC for one more packet only wakes it when the
  // controller is still responsive, which is exactly the condition that is
  // absent during the field failure this shutdown path must handle.
  impl_->packet_thread_run_.store(false);
  if (impl_->read_fd_ >= 0) {
    ::close(impl_->read_fd_);
    impl_->read_fd_ = -1;
  }
  auto port = impl_->serial_driver_->port();
  if (port && port->is_open()) {
    try {
      port->close();
    } catch (const std::exception & error) {
      if (impl_->error_handler_) {
        impl_->error_handler_(
          std::string("Serial close failed: ") + error.what());
      }
    }
  }
  if (impl_->packet_thread_ && impl_->packet_thread_->joinable()) {
    impl_->packet_thread_->join();
  }
  impl_->packet_thread_.reset();
  impl_->reset_buffer();
}

bool VescInterface::isConnected() const
{
  auto port = impl_->serial_driver_->port();
  if (port) {
    return port->is_open();
  } else {
    return false;
  }
}

void VescInterface::send(const VescPacket & packet)
{
  if (!isConnected()) {
    throw SerialException("Cannot send because the serial port is closed");
  }
  impl_->serial_driver_->port()->async_send(packet.frame());
}

void VescInterface::sendBlocking(const VescPacket & packet)
{
  if (!isConnected()) {
    throw SerialException("Cannot send because the serial port is closed");
  }
  impl_->serial_driver_->port()->send(packet.frame());
}

void VescInterface::requestFWVersion()
{
  send(VescPacketRequestFWVersion());
}

void VescInterface::requestState()
{
  send(VescPacketRequestValues());
}

void VescInterface::setDutyCycle(double duty_cycle)
{
  send(VescPacketSetDuty(duty_cycle));
}

void VescInterface::setDutyCycleBlocking(double duty_cycle)
{
  sendBlocking(VescPacketSetDuty(duty_cycle));
}

void VescInterface::setCurrent(double current)
{
  send(VescPacketSetCurrent(current));
}

void VescInterface::setBrake(double brake)
{
  send(VescPacketSetCurrentBrake(brake));
}

void VescInterface::setSpeed(double speed)
{
  send(VescPacketSetRPM(speed));
}

void VescInterface::setPosition(double position)
{
  send(VescPacketSetPos(position));
}

void VescInterface::setServo(double servo)
{
  send(VescPacketSetServoPos(servo, impl_->legacy_protocol_.load()));
}

void VescInterface::requestImuData()
{
  send(VescPacketRequestImu());
}

void VescInterface::setLegacyProtocol(bool enabled)
{
  impl_->legacy_protocol_.store(enabled);
}

}  // namespace vesc_driver
