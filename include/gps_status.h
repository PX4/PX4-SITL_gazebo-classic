#pragma once

#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/system/error_code.hpp>
#include <boost/system/system_error.hpp>
#include <boost/bind.hpp>
#include <boost/thread.hpp>

#include <string>
#include <vector>
#include <sstream>
#include <functional>

// IDK about this
typedef boost::shared_ptr<boost::asio::serial_port> serial_port_ptr;

#define SERIAL_PORT_READ_BUF_SIZE 256

class SerialPort
{
 protected:
    boost::asio::io_service io_service;
    serial_port_ptr port;
    boost::mutex mutex;

    char read_buf_raw[SERIAL_PORT_READ_BUF_SIZE];
    std::string read_buf_str;

    char end_of_line_char;

    std::function<void(const std::string &)> cb = nullptr;

 private:
    SerialPort(const SerialPort &p);
    SerialPort &operator=(const SerialPort &p);

 public:
    SerialPort(void);
    ~SerialPort(void);

    void setEOLChar(const char &c);

    bool start(const char *com_port_name, int baud_rate=9600);
    void stop();

    int write_some(const std::string &buf);
    int write_some(const char *buf, const size_t &size);

    void setCallback(std::function<void(const std::string &)> &&func);

 protected:
    void async_read_some();
    void on_receive(const boost::system::error_code& ec, size_t bytes_transferred);
    void on_receive(const std::string &data);
    void parse_line(const std::string &data);
};

enum class nmea_type
{
    gsv,
    unknown
};

struct satellite
{
    uint8_t prn;
    uint8_t elevation;
    uint8_t azimuth;
    uint8_t snr;
};

struct nmea_sentence
{
    std::vector<std::string> fields;
    std::string sentence = {};
    nmea_type type;
    bool valid;

    nmea_type strToType(const std::string &input)
    {
        if (input == "GSV")
            return nmea_type::gsv;

        return nmea_type::unknown;
    }

    bool validate_checksum(const std::string &input)
    {
        size_t idx = input.find_last_of('*');

        uint8_t checksum = 0;

        for (uint8_t i = 1; i < idx; ++i)
        {
            checksum ^= input.at(i);
        }

        std::stringstream ss;

        ss << std::hex << std::uppercase << std::setfill('0') << std::setw(2) << static_cast<uint32_t>(checksum);

        return input.compare(idx + 1, 2, ss.str(), 0, 2) == 0;
    }

    nmea_sentence(const std::string &input)
    {
        valid = validate_checksum(input);

        // Extract the field
        if (valid)
        {
            std::size_t delim = input.find_first_of(',');

            sentence = input;
            type =  strToType(input.substr(delim - 3, 3));

            if (type == nmea_type::unknown)
            {
                valid = false;
                return;
            }


            while (delim != std::string::npos)
            {
                size_t next_delim = input.find_first_of(',', delim + 1);
                size_t len;

                if (next_delim == std::string::npos)
                    len = input.find_last_of('*') - delim - 1;
                else
                    len = next_delim - delim - 1;

                if (len == 0)
                    fields.push_back("NULL");
                else
                    fields.push_back(input.substr(delim + 1, len));

                delim = next_delim;
            }

        }
    }
};

struct gsv_sentence : nmea_sentence
{
    std::vector<satellite> satellite_info = {};
    uint8_t tot_msg_num,
        msg_num,
        sat_count;

 gsv_sentence(const std::string &input)
     : nmea_sentence(input)
    {
        if (valid)
        {
            valid = false;

            // Parse through gsv fields
            tot_msg_num = std::stoul(fields[0]);
            msg_num = std::stoul(fields[1]);
            sat_count = std::stoul(fields[2]);

            uint8_t num_entries = (fields.size() - 3)/4;
            satellite_info.reserve(num_entries);

            for (uint8_t i = 0; i < num_entries; ++i)
            {
                    satellite sat;
                    std::string field = fields[4*i + 3];
                    if (field == "NULL")
                        continue;
                    sat.prn = std::stoul(field);

                    field = fields[4*i + 4];
                    if (field == "NULL")
                        continue;
                    sat.elevation = std::stoul(field);

                    field = fields[4*i + 5];
                    if (field == "NULL")
                        continue;
                    sat.azimuth = std::stoul(field);

                    field = fields[4*i + 6];
                    if (field == "NULL")
                        sat.snr = 0.0;
                    else
                        sat.snr = std::stoul(field);


                    satellite_info.emplace_back(sat);
            }

            // If we get to here then we successfully parsed the GSV string
            valid = true;
        }
    }
};
