#ifndef ARDUINO_SERIAL_COMMS_HPP_
#define ARDUINO_SERIAL_COMMS_HPP_

#include <libserial/SerialPort.h>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

LibSerial::BaudRate convert_baud_rate(int baud_rate)
{
  // Just handle some common baud rates
  switch (baud_rate)
  {
    case 1200: return LibSerial::BaudRate::BAUD_1200;
    case 1800: return LibSerial::BaudRate::BAUD_1800;
    case 2400: return LibSerial::BaudRate::BAUD_2400;
    case 4800: return LibSerial::BaudRate::BAUD_4800;
    case 9600: return LibSerial::BaudRate::BAUD_9600;
    case 19200: return LibSerial::BaudRate::BAUD_19200;
    case 38400: return LibSerial::BaudRate::BAUD_38400;
    case 57600: return LibSerial::BaudRate::BAUD_57600;
    case 115200: return LibSerial::BaudRate::BAUD_115200;
    case 230400: return LibSerial::BaudRate::BAUD_230400;
    default:
      return LibSerial::BaudRate::BAUD_57600;
  }
}

class ArduinoSerialComms
{
    public:

        ArduinoSerialComms() = default;

        bool isConnected()
        {
            return serial_conn_.IsOpen();
        }

        void connect(const std::string &serial_device, int32_t baud_rate)
        {
            serial_conn_.Open(serial_device);
            serial_conn_.SetBaudRate(convert_baud_rate(baud_rate));
        }

        void disconnect()
        {
            serial_conn_.Close();
        }

        void send_msg(json &msg)
        {
            auto string = msg.dump();

            serial_conn_.FlushIOBuffers();
            serial_conn_.Write(string);
        }

    private: 
        LibSerial::SerialPort serial_conn_;
};

#endif // ARDUINO_SERIAL_COMMS_HPP_