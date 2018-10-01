// *****************************************************************************
//
// Copyright (c) 2018, Southwest Research Institute® (SwRI®)
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of Southwest Research Institute® (SwRI®) nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL SOUTHWEST RESEARCH INSTITUTE BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// *****************************************************************************

#ifndef INFUSE_NOVATEL_GPS_DRIVER_BESTUTM_H
#define INFUSE_NOVATEL_GPS_DRIVER_BESTUTM_H

// #include <infuse_novatel_gps_msgs/NovatelUtmPosition.h>
#include <infuse_msgs/asn1_bitstream.h>

#include <infuse_novatel_gps_driver/parsers/parsing_utils.h>
#include <infuse_novatel_gps_driver/parsers/message_parser.h>

#include <fstream>

namespace infuse_novatel_gps_driver
{
  // class BestutmParser : public MessageParser<infuse_novatel_gps_msgs::NovatelUtmPositionPtr>
  class BestutmParser : public MessageParser<infuse_msgs::asn1_bitstreamPtr>
  {
  public:
    BestutmParser();
    virtual ~BestutmParser();
    uint32_t GetMessageId() const override;

    const std::string GetMessageName() const override;

    // infuse_novatel_gps_msgs::NovatelUtmPositionPtr ParseBinary(const BinaryMessage& bin_msg) throw(ParseException) override;
    infuse_msgs::asn1_bitstreamPtr ParseBinary(const BinaryMessage& bin_msg, long long time_usec) throw(ParseException);

    // infuse_novatel_gps_msgs::NovatelUtmPositionPtr ParseAscii(const NovatelSentence& sentence) throw(ParseException) override;
    infuse_msgs::asn1_bitstreamPtr ParseAscii(const NovatelSentence& sentence, long long time_usec) throw(ParseException);

    static constexpr uint16_t MESSAGE_ID = 726;
    static constexpr size_t BINARY_LENGTH = 80;
    static constexpr size_t ASCII_LENGTH = 23;
    static const std::string MESSAGE_NAME;

    std::ofstream utm_data_fs;
  };
}

#endif //INFUSE_NOVATEL_GPS_DRIVER_BESTUTM_H
