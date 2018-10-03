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

#include <infuse_novatel_gps_driver/parsers/bestutm.h>

#include <infuse_novatel_gps_driver/parsers/header.h>

#include <boost/make_shared.hpp>

#include <infuse_asn1_types/TransformWithCovariance.h>

#include <time.h>

namespace infuse_novatel_gps_driver
{
  const std::string BestutmParser::MESSAGE_NAME = "BESTUTM";

  BestutmParser::BestutmParser()
  {
    utm_data_fs.open("/home/infuse/tests/gps_utm_data.txt");
    utm_data_fs.precision(16);
    utm_data_fs << "#ROSTime parentTime childTime x y z qw qx qy qz sig_x sig_y sig_z" << std::endl;
  }

  BestutmParser::~BestutmParser()
  {
    utm_data_fs.close();
  }

  uint32_t BestutmParser::GetMessageId() const
  {
    return MESSAGE_ID;
  }

  const std::string BestutmParser::GetMessageName() const
  {
    return MESSAGE_NAME;
  }

  // infuse_novatel_gps_msgs::NovatelUtmPositionPtr BestutmParser::ParseBinary(const BinaryMessage& bin_msg) throw(ParseException)
  infuse_msgs::asn1_bitstreamPtr BestutmParser::ParseBinary(const BinaryMessage& bin_msg, long long time_usec) throw(ParseException)
  {
    if (bin_msg.data_.size() != BINARY_LENGTH)
    {
      std::stringstream error;
      error << "Unexpected BESTUTM message length: " << bin_msg.data_.size();
      throw ParseException(error.str());
    }

    asn1SccTransformWithCovariance asn1Transform;
    asn1SccTransformWithCovariance_Initialize(&asn1Transform);

    asn1Transform.metadata.msgVersion = transformWithCovariance_version;

    sprintf((char*)asn1Transform.metadata.producerId.arr, "novatel_gps_driver");
    asn1Transform.metadata.producerId.nCount = strlen((char*)asn1Transform.metadata.producerId.arr) + 1;

    sprintf((char*)asn1Transform.metadata.parentFrameId.arr, "GlobalTerrainFrame");
    asn1Transform.metadata.parentFrameId.nCount = strlen((char*)asn1Transform.metadata.parentFrameId.arr) + 1;
    asn1Transform.metadata.parentTime.microseconds = time_usec;
    asn1Transform.metadata.parentTime.usecPerSec = 1000000;

    sprintf((char*)asn1Transform.metadata.childFrameId.arr, "GPSFrame");
    asn1Transform.metadata.childFrameId.nCount = strlen((char*)asn1Transform.metadata.childFrameId.arr) + 1;
    asn1Transform.metadata.childTime.microseconds  = time_usec;
    asn1Transform.metadata.childTime.usecPerSec = 1000000;

    double easting = ParseDouble(&bin_msg.data_[24]);
    double northing = ParseDouble(&bin_msg.data_[16]);
    double height = ParseDouble(&bin_msg.data_[32]);

    asn1Transform.data.translation.arr[0] = easting;
    asn1Transform.data.translation.arr[1] = northing;
    asn1Transform.data.translation.arr[2] = height;

    asn1Transform.data.orientation.arr[0] = 0;
    asn1Transform.data.orientation.arr[1] = 0;
    asn1Transform.data.orientation.arr[2] = 0;
    asn1Transform.data.orientation.arr[3] = 1;

    // TODO translate or_pose_estimator covariance to envire covariance
    for(int i = 0; i < 6; i++)
    {
        for(int j = 0; j < 6; j++)
        {
            asn1Transform.data.cov.arr[i].arr[j] = 0;
        }
    }

    double easting_sigma = ParseFloat(&bin_msg.data_[52]);
    double northing_sigma = ParseFloat(&bin_msg.data_[48]);
    double height_sigma = ParseFloat(&bin_msg.data_[56]);

    asn1Transform.data.cov.arr[0].arr[0] = easting_sigma*easting_sigma;
    asn1Transform.data.cov.arr[1].arr[1] = northing_sigma*northing_sigma;
    asn1Transform.data.cov.arr[2].arr[2] = height_sigma*height_sigma;
    // to have a well defined cov matrix :
    for(int i = 3; i < 6; i++)
        asn1Transform.data.cov.arr[i].arr[i] = 1e-6;

    byte buffer[asn1SccTransformWithCovariance_REQUIRED_BYTES_FOR_ENCODING];

    BitStream bstream;
    BitStream_Init(&bstream, buffer, asn1SccTransformWithCovariance_REQUIRED_BYTES_FOR_ENCODING);

    int errorCode;
    flag encoding_ok = asn1SccTransformWithCovariance_Encode(&asn1Transform, &bstream, &errorCode, TRUE);
    if(!encoding_ok)
    {
      std::stringstream error;
      error << "asn1SccTransformWithCovariance encoding error: " << errorCode;
      throw ParseException(error.str());
    }

    infuse_msgs::asn1_bitstreamPtr ros_msg = boost::make_shared<infuse_msgs::asn1_bitstream>();

    // Fill ROS header (needed for visualization)
    ros_msg->header.frame_id = "GlobalTerrainFrame";
    ros_msg->header.stamp = ros::Time::now(); // FIXME use same time as transform

    // Fill the type (needed to decode into the same type upon reception)
    ros_msg->type = "TransformWithCovariance";

    ros_msg->serialization_method = 0; //uPER

    // Fill bitstream
    ros_msg->data.assign(bstream.buf, bstream.buf + BitStream_GetLength(&bstream));


    // infuse_novatel_gps_msgs::NovatelUtmPositionPtr ros_msg =
    //     boost::make_shared<infuse_novatel_gps_msgs::NovatelUtmPosition>();
    // HeaderParser header_parser;
    // ros_msg->novatel_msg_header = header_parser.ParseBinary(bin_msg);
    // ros_msg->novatel_msg_header.message_name = MESSAGE_NAME;

    // uint16_t solution_status = ParseUInt16(&bin_msg.data_[0]);
    // if (solution_status > MAX_SOLUTION_STATUS)
    // {
    //   std::stringstream error;
    //   error << "Unknown solution status: " << solution_status;
    //   throw ParseException(error.str());
    // }
    // ros_msg->solution_status = SOLUTION_STATUSES[solution_status];
    // uint16_t pos_type = ParseUInt16(&bin_msg.data_[4]);
    // if (pos_type > MAX_POSITION_TYPE)
    // {
    //   std::stringstream error;
    //   error << "Unknown position type: " << pos_type;
    //   throw ParseException(error.str());
    // }
    // ros_msg->position_type = POSITION_TYPES[pos_type];
    // ros_msg->lon_zone_number = ParseUInt32(&bin_msg.data_[8]);
    // ros_msg->lat_zone_letter = (char)ParseUInt32(&bin_msg.data_[12]);
    // ros_msg->northing = ParseDouble(&bin_msg.data_[16]);
    // ros_msg->easting = ParseDouble(&bin_msg.data_[24]);
    // ros_msg->height = ParseDouble(&bin_msg.data_[32]);
    // ros_msg->undulation = ParseFloat(&bin_msg.data_[40]);
    // uint16_t datum_id = ParseUInt16(&bin_msg.data_[44]);
    // if (datum_id > MAX_DATUM)
    // {
    //   std::stringstream error;
    //   error << "Unknown datum: " << datum_id;
    //   throw ParseException(error.str());
    // }
    // ros_msg->datum_id = DATUMS[datum_id];
    // ros_msg->northing_sigma = ParseFloat(&bin_msg.data_[48]);
    // ros_msg->easting_sigma = ParseFloat(&bin_msg.data_[52]);
    // ros_msg->height_sigma = ParseFloat(&bin_msg.data_[56]);
    // ros_msg->base_station_id.resize(4);
    // std::copy(&bin_msg.data_[60], &bin_msg.data_[64], &ros_msg->base_station_id[0]);
    // ros_msg->diff_age = ParseFloat(&bin_msg.data_[64]);
    // ros_msg->solution_age = ParseFloat(&bin_msg.data_[68]);
    // ros_msg->num_satellites_tracked = bin_msg.data_[72];
    // ros_msg->num_satellites_used_in_solution = bin_msg.data_[73];
    // ros_msg->num_gps_and_glonass_l1_used_in_solution = bin_msg.data_[74];
    // ros_msg->num_gps_and_glonass_l1_and_l2_used_in_solution = bin_msg.data_[75];
    // GetExtendedSolutionStatusMessage(bin_msg.data_[77],
    //                                  ros_msg->extended_solution_status);
    // GetSignalsUsed(bin_msg.data_[78], ros_msg->signal_mask);

    utm_data_fs << time_usec << " "
                << asn1Transform.metadata.parentTime.microseconds << " "
                << asn1Transform.metadata.childTime.microseconds << " "
                << asn1Transform.data.translation.arr[0] << " " // x
                << asn1Transform.data.translation.arr[1] << " " // y
                << asn1Transform.data.translation.arr[2] << " " // z
                << asn1Transform.data.orientation.arr[3] << " " // qw
                << asn1Transform.data.orientation.arr[0] << " " // qx
                << asn1Transform.data.orientation.arr[1] << " " // qy
                << asn1Transform.data.orientation.arr[2] << " " // qz
                << easting_sigma                         << " " // sig_x
                << northing_sigma                        << " " // sig_y
                << height_sigma                                 // sig_z
                << std::endl;

    return ros_msg;
  }

  // infuse_novatel_gps_msgs::NovatelUtmPositionPtr BestutmParser::ParseAscii(const NovatelSentence& sentence) throw(ParseException)
  infuse_msgs::asn1_bitstreamPtr BestutmParser::ParseAscii(const NovatelSentence& sentence, long long time_usec) throw(ParseException)
  {
    // infuse_msgs::asn1_bitstreamPtr msg =
    //     boost::make_shared<infuse_msgs::asn1_bitstream>();
    // HeaderParser h_parser;
    // msg->novatel_msg_header = h_parser.ParseAscii(sentence);

    if (sentence.body.size() != ASCII_LENGTH)
    {
      std::stringstream error;
      error << "Unexpected number of BESTUTM message fields: " << sentence.body.size();
      throw ParseException(error.str());
    }

    bool valid = true;

    // msg->solution_status = sentence.body[0];
    // msg->position_type = sentence.body[1];
    // valid = valid && ParseUInt32(sentence.body[2], msg->lon_zone_number);
    // msg->lat_zone_letter = sentence.body[3];
    // valid = valid && ParseDouble(sentence.body[4], msg->northing);
    // valid = valid && ParseDouble(sentence.body[5], msg->easting);
    // valid = valid && ParseDouble(sentence.body[6], msg->height);
    // valid = valid && ParseFloat(sentence.body[7], msg->undulation);
    // msg->datum_id = sentence.body[8];
    // valid = valid && ParseFloat(sentence.body[9], msg->northing_sigma);
    // valid = valid && ParseFloat(sentence.body[10], msg->easting_sigma);
    // valid = valid && ParseFloat(sentence.body[11], msg->height_sigma);
    // msg->base_station_id = sentence.body[12];
    // valid = valid && ParseFloat(sentence.body[13], msg->diff_age);
    // valid = valid && ParseFloat(sentence.body[14], msg->solution_age);
    // valid = valid && ParseUInt8(sentence.body[15], msg->num_satellites_tracked);
    // valid = valid && ParseUInt8(sentence.body[16], msg->num_satellites_used_in_solution);
    // valid = valid && ParseUInt8(sentence.body[17], msg->num_gps_and_glonass_l1_used_in_solution);
    // valid = valid && ParseUInt8(sentence.body[18], msg->num_gps_and_glonass_l1_and_l2_used_in_solution);

    // // skip reserved field
    // uint32_t extended_solution_status = 0;
    // valid = valid && ParseUInt32(sentence.body[20], extended_solution_status, 16);
    // GetExtendedSolutionStatusMessage(
    //     extended_solution_status, msg->extended_solution_status);

    // // skip reserved field (Galileo and BeiDou sig mask)
    // uint32_t signal_mask = 0;
    // valid = valid && ParseUInt32(sentence.body[22], signal_mask, 16);
    // GetSignalsUsed(signal_mask, msg->signal_mask);

    double northing;
    double easting;
    double height;
    float northing_sigma;
    float easting_sigma;
    float height_sigma;
    valid = valid && ParseDouble(sentence.body[4], northing);
    valid = valid && ParseDouble(sentence.body[5], easting);
    valid = valid && ParseDouble(sentence.body[6], height);
    valid = valid && ParseFloat(sentence.body[9], northing_sigma);
    valid = valid && ParseFloat(sentence.body[10], easting_sigma);
    valid = valid && ParseFloat(sentence.body[11], height_sigma);

    if (!valid)
    {
      throw ParseException("Invalid field in BESTUTM message");
    }

    asn1SccTransformWithCovariance asn1Transform;
    asn1SccTransformWithCovariance_Initialize(&asn1Transform);

    asn1Transform.metadata.msgVersion = transformWithCovariance_version;

    sprintf((char*)asn1Transform.metadata.producerId.arr, "novatel_gps_driver");
    asn1Transform.metadata.producerId.nCount = strlen((char*)asn1Transform.metadata.producerId.arr) + 1;

    sprintf((char*)asn1Transform.metadata.parentFrameId.arr, "GlobalTerrainFrame");
    asn1Transform.metadata.parentFrameId.nCount = strlen((char*)asn1Transform.metadata.parentFrameId.arr) + 1;
    asn1Transform.metadata.parentTime.microseconds = time_usec;
    asn1Transform.metadata.parentTime.usecPerSec = 1000000;

    sprintf((char*)asn1Transform.metadata.childFrameId.arr, "GPSFrame");
    asn1Transform.metadata.childFrameId.nCount = strlen((char*)asn1Transform.metadata.childFrameId.arr) + 1;
    asn1Transform.metadata.childTime.microseconds  = time_usec;
    asn1Transform.metadata.childTime.usecPerSec = 1000000;

    asn1Transform.data.translation.arr[0] = easting;
    asn1Transform.data.translation.arr[1] = northing;
    asn1Transform.data.translation.arr[2] = height;

    asn1Transform.data.orientation.arr[0] = 0;
    asn1Transform.data.orientation.arr[1] = 0;
    asn1Transform.data.orientation.arr[2] = 0;
    asn1Transform.data.orientation.arr[3] = 1;

    // TODO translate or_pose_estimator covariance to envire covariance
    for(int i = 0; i < 6; i++)
    {
        for(int j = 0; j < 6; j++)
        {
            asn1Transform.data.cov.arr[i].arr[j] = 0;
        }
    }

    asn1Transform.data.cov.arr[0].arr[0] = ((double)easting_sigma)*((double)easting_sigma);
    asn1Transform.data.cov.arr[1].arr[1] = ((double)northing_sigma)*((double)northing_sigma);
    asn1Transform.data.cov.arr[2].arr[2] = ((double)height_sigma)*((double)height_sigma);
    // to have a well defined cov matrix :
    for(int i = 3; i < 6; i++)
        asn1Transform.data.cov.arr[i].arr[i] = 1e-6;

    byte buffer[asn1SccTransformWithCovariance_REQUIRED_BYTES_FOR_ENCODING];

    BitStream bstream;
    BitStream_Init(&bstream, buffer, asn1SccTransformWithCovariance_REQUIRED_BYTES_FOR_ENCODING);

    int errorCode;
    flag encoding_ok = asn1SccTransformWithCovariance_Encode(&asn1Transform, &bstream, &errorCode, TRUE);
    if(!encoding_ok)
    {
      std::stringstream error;
      error << "asn1SccTransformWithCovariance encoding error: " << errorCode;
      throw ParseException(error.str());
    }

    infuse_msgs::asn1_bitstreamPtr ros_msg = boost::make_shared<infuse_msgs::asn1_bitstream>();

    // Fill ROS header (needed for visualization)
    ros_msg->header.frame_id = "GlobalTerrainFrame";
    ros_msg->header.stamp = ros::Time::now(); // FIXME use same time as transform

    // Fill the type (needed to decode into the same type upon reception)
    ros_msg->type = "TransformWithCovariance";

    ros_msg->serialization_method = 0; //uPER

    // Fill bitstream
    ros_msg->data.assign(bstream.buf, bstream.buf + BitStream_GetLength(&bstream));


    return ros_msg;
  }
};
