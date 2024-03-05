#include <iostream>

#include "gps_status.h"

SerialPort::SerialPort(void) : end_of_line_char('\n')
{

}

SerialPort::~SerialPort(void)
{
    stop();
}

void SerialPort::setEOLChar(const char &c)
{
    end_of_line_char = c;
}

bool SerialPort::start(const char *com_port_name, int baud_rate)
{
    boost::system::error_code ec;

    if (port)
    {
        std::cerr << "port is already opened\n";
        return false;
    }

    port = serial_port_ptr(new boost::asio::serial_port(io_service));
    port->open(com_port_name, ec);
    if (ec)
    {
        std::cerr << "port \"" << com_port_name << "\" failed to open\n"
                  << ec.message().c_str() << std::endl;

        return false;
    }

    port->set_option(boost::asio::serial_port_base::baud_rate(baud_rate));
    port->set_option(boost::asio::serial_port_base::character_size(8));
    port->set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
    port->set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
    port->set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));

    boost::thread t(boost::bind(&boost::asio::io_service::run, &io_service));

    async_read_some();

    return true;
}

void SerialPort::stop()
{
    boost::mutex::scoped_lock look(mutex);

    if (port)
    {
        port->cancel();
        port->close();
        port.reset();
    }

    io_service.stop();
    io_service.reset();
}

int SerialPort::write_some(const std::string &buf)
{
    return write_some(buf.c_str(), buf.size());
}

int SerialPort::write_some(const char *buf, const size_t &size)
{
    boost::system::error_code ec;

    if (!port)
        return -1;

    if (size == 0)
        return 0;

    return port->write_some(boost::asio::buffer(buf, size), ec);
}

void SerialPort::async_read_some()
{
    if (port.get() == NULL || !port->is_open())
        return;

    port->async_read_some(
                          boost::asio::buffer(read_buf_raw, SERIAL_PORT_READ_BUF_SIZE),
                          boost::bind(
                                      &SerialPort::on_receive,
                                      this, boost::asio::placeholders::error,
                                      boost::asio::placeholders::bytes_transferred)
                          );
}

void SerialPort::on_receive(const boost::system::error_code &ec, size_t bytes_transferred)
{
    boost::mutex::scoped_lock look(mutex);

    if (port.get() == NULL || !port->is_open())
        return;

    if (ec)
    {
        async_read_some();
        return;
    }

    for (unsigned int i = 0; i < bytes_transferred; ++i)
    {
        char c = read_buf_raw[i];
        if (c == end_of_line_char)
        {
            this->on_receive(read_buf_str);
            read_buf_str.clear();
        }
        else
        {
            read_buf_str += c;
        }
    }

    async_read_some();
}

void SerialPort::setCallback(std::function<void(const std::string &)> &&func)
{
    cb = func;
}

void SerialPort::on_receive(const std::string &data)
{
    if (cb)
        cb(data);
}
