/*
 * Copyright (C) 2021 Prensilia s.r.l.
 *
 * Redistribution and use in source and binary forms, with or without 
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, 
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, 
 *    this list of conditions and the following disclaimer in the documentation 
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors 
 *    may be used to endorse or promote products derived from this software 
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE 
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "mia_hand_driver/cpp_driver.h"

namespace mia_hand
{
CppDriver::CppDriver():
  serial_port_(&finger_data_mtx_, &reply_mtx_, &is_connected_),
  connection_trd_on_(true),
  serial_trd_on_(true),
  is_checking_on_(false),
  is_paused_(false),
  stream_flag_bits_(0x00)
{
  /* Starting serial polling detached thread.
   */
  serial_poll_trd_ = std::thread(&CppDriver::pollSerialPort, this);
  serial_poll_trd_.detach();

  /* Starting connection checking detached thread.
   */
  check_connection_trd_ = std::thread(&CppDriver::checkConnection, this);
  check_connection_trd_.detach();
}

CppDriver::~CppDriver()
{
  /* Ending detached threads by clearing related flags.
   */
  connection_trd_on_ = false;
  serial_trd_on_     = false;
}

bool CppDriver::connectToPort(uint16_t port_num)
{
  bool is_port_opened = serial_port_.open(port_num);

  return is_port_opened;
}

bool CppDriver::disconnect()
{
  bool is_port_closed = serial_port_.close();

  if (is_port_closed)
  {
    connection_mtx_.lock();
    is_connected_ = false;
    connection_mtx_.unlock();
  }

  return is_port_closed;
}

bool CppDriver::isConnected()
{
  connection_mtx_.lock();
  bool is_connected = is_connected_;
  connection_mtx_.unlock();

  return is_connected;
}

void CppDriver::pause()
{
  is_paused_ = true;  // Signaling paused state.

  /* Stopping motors.
   */
  setMotorSpe(0, 0);  // Thumb.
  setMotorSpe(1, 0);  // Mrl.
  setMotorSpe(2, 0);  // Index.

  return;
}

void CppDriver::play()
{
  is_paused_ = false;  // Signaling normal operation.

  return;
}

bool CppDriver::isPaused()
{
  return is_paused_;
}

void CppDriver::setMotorPos(uint8_t mot_id, int16_t mot_pos)
{ 
  if (!is_paused_)
  {
    std::string pos_cmd = "@" + std::to_string(mot_id + 1) + "P+000040......";

    if (mot_pos < 0)
    {
      pos_cmd.replace(3, 1, "-");
      mot_pos = -mot_pos;
    }

    if (mot_pos > 999)
    {
      pos_cmd.replace(5, 3, "999");
    }
    else
    {
      pos_cmd.replace(5, 3, numToStr(mot_pos, 3));
    }

    serial_port_.sendCommand(pos_cmd);
  }

  return;
}

void CppDriver::setMotorSpe(uint8_t mot_id, int16_t mot_spe)
{
  if (!is_paused_)
  {
    std::string spe_cmd = "@" + std::to_string(mot_id + 1) + "S+....0050....";

    if (mot_spe < 0)
    {
      spe_cmd.replace(3, 1, "-");
      mot_spe = -mot_spe;
    }

    if (mot_spe > 99)
    {
      spe_cmd.replace(8, 2, "99");
    }
    else
    {
      spe_cmd.replace(8, 2, numToStr(mot_spe, 2));
    }

    serial_port_.sendCommand(spe_cmd);
  }

  return;
}

void CppDriver::setFingerFor(uint8_t mot_id, int16_t fin_for)
{ 
  if (!is_paused_)
  {
    std::string for_cmd = "@" + std::to_string(mot_id + 1) + "F+....0080....";

    if (fin_for > 99)
    {
      for_cmd.replace(8, 2, "99");
    }
    else if (fin_for > 0)
    {
      for_cmd.replace(8, 2, numToStr(fin_for, 2));
    }
    else
    {
      // Default empty case
    }

    serial_port_.sendCommand(for_cmd);
  }

  return;
}

int16_t CppDriver::getMotorPos(uint8_t fin_id)
{
  int16_t mot_pos;

  finger_data_mtx_.lock();

  switch (fin_id)
  {
    case 0:

      mot_pos = thumb_info_.mot_pos;

    break; 

    case 1:

      mot_pos = mrl_info_.mot_pos;

    break;

    case 2:

      mot_pos = index_info_.mot_pos;

    break;

    default:

    break;
  }

  finger_data_mtx_.unlock();

  return mot_pos;
}

int16_t CppDriver::getMotorSpe(uint8_t fin_id)
{
  int16_t mot_spe;

  finger_data_mtx_.lock();

  switch (fin_id)
  {
    case 0:

      mot_spe = thumb_info_.mot_spe;

    break; 

    case 1:

      mot_spe = mrl_info_.mot_spe;

    break;

    case 2:

      mot_spe = index_info_.mot_spe;

    break;

    default:

    break;
  }

  finger_data_mtx_.unlock();

  return mot_spe;
}

int16_t CppDriver::getMotorCur(uint8_t fin_id)
{
  int16_t mot_cur;

  finger_data_mtx_.lock();

  switch (fin_id)
  {
    case 0:

      mot_cur = thumb_info_.mot_cur;

    break; 

    case 1:

      mot_cur = mrl_info_.mot_cur;

    break;

    case 2:

      mot_cur = index_info_.mot_cur;

    break;

    default:

    break;
  }

  finger_data_mtx_.unlock();

  return mot_cur;
}

void CppDriver::getFingerSgRaw(uint8_t sens_id, int16_t& output)
{
  finger_data_mtx_.lock();

  switch (sens_id)
  {
    case 0:  // Mrl normal

      output = mrl_info_.fin_sg_raw[0];

    break; 

    case 1:  // Index tangential

      output = index_info_.fin_sg_raw[1];

    break;

    case 2:  // Index normal

      output = index_info_.fin_sg_raw[0];

    break;

    case 3:  // Thumb normal

      output = thumb_info_.fin_sg_raw[0];

    break;

    case 4:  // Thumb tangential

      output = thumb_info_.fin_sg_raw[1];

    break;

    case 5:  // Mrl tangential

      output = mrl_info_.fin_sg_raw[1];

    break;

    default:

    break;
  }

  finger_data_mtx_.unlock();

  return;
}

void CppDriver::openGrasp(char grasp_id)
{
  if (!is_paused_)
  {
    std::string grasp_cmd = "@AG" + std::string(1, grasp_id) + "a10050......";
    serial_port_.sendCommand(grasp_cmd);
  }

  return;
}

void CppDriver::closeGrasp(char grasp_id)
{
  if (!is_paused_)
  {
    std::string grasp_cmd = "@AG" + std::string(1, grasp_id) + "A10050......";
    serial_port_.sendCommand(grasp_cmd);
  }

  return;
}

void CppDriver::closeGrasp(char grasp_id, int16_t close_percent)
{
  if (!is_paused_)
  {
    std::string grasp_cmd = "@AG" + std::string(1, grasp_id) + "M00050......";

    if (close_percent > 99)
    {
      grasp_cmd.replace(5, 3, "100");
    }
    else if (close_percent > 0)
    {
      grasp_cmd.replace(6, 2, numToStr(close_percent, 2));
    }
    else
    {
    }

    serial_port_.sendCommand(grasp_cmd);
  }

  return;
}

void CppDriver::setThuGraspRef(char grasp_id, int16_t rest, int16_t pos,
                                int16_t delay)
{
  std::string grasp_ref_cmd = "@1G" + std::string(1, grasp_id) + "+000+000+000";

  if (rest > 0)
  {
    grasp_ref_cmd.replace(5, 3, numToStr(rest, 3));
  }

  if (pos > 0)
  {
    grasp_ref_cmd.replace(9, 3, numToStr(pos, 3));
  }

  if (delay > 99)
  {
    grasp_ref_cmd.replace(13, 1, "1");
  }
  else if (delay > 0)
  {
    grasp_ref_cmd.replace(13, 3, numToStr(delay, 3));
  }
  else
  {

  }

  serial_port_.sendCommand(grasp_ref_cmd);

  return;
}

void CppDriver::setIndGraspRef(char grasp_id, int16_t rest, int16_t pos,
                                int16_t delay)
{
  std::string grasp_ref_cmd = "@3G" + std::string(1, grasp_id) + "+000+000+000";

  if (rest > 0)
  {
    grasp_ref_cmd.replace(5, 3, numToStr(rest, 3));
  }

  if (pos > 0)
  {
    grasp_ref_cmd.replace(9, 3, numToStr(pos, 3));
  }

  if (delay > 99)
  {
    grasp_ref_cmd.replace(13, 1, "1");
  }
  else if (delay > 0)
  {
    grasp_ref_cmd.replace(13, 3, numToStr(delay, 3));
  }
  else
  {

  }

  serial_port_.sendCommand(grasp_ref_cmd);

  return;
}

void CppDriver::setMrlGraspRef(char grasp_id, int16_t rest, int16_t pos,
                                int16_t delay)
{
  std::string grasp_ref_cmd = "@2G" + std::string(1, grasp_id) + "+000+000+000";

  if (rest > 0)
  {
    grasp_ref_cmd.replace(5, 3, numToStr(rest, 3));
  }

  if (pos > 0)
  {
    grasp_ref_cmd.replace(9, 3, numToStr(pos, 3));
  }

  if (delay > 99)
  {
    grasp_ref_cmd.replace(13, 1, "1");
  }
  else if (delay > 0)
  {
    grasp_ref_cmd.replace(13, 3, numToStr(delay, 3));
  }
  else
  {

  }

  serial_port_.sendCommand(grasp_ref_cmd);

  return;
}

void CppDriver::switchPosStream(bool b_on_off)
{
  std::string stream_cmd = "@ADP" + std::to_string(b_on_off ? 1 : 0)
                         + "...........";

  serial_port_.sendCommand(stream_cmd); 

  if (true == b_on_off)
  {
    stream_flag_bits_ |= 0x01;  // Signaling streaming active.
  }
  else
  {
    stream_flag_bits_ &= 0xFFFE;  // Signaling streaming not active.
  }

  return;
}

void CppDriver::switchSpeStream(bool b_on_off)
{
  std::string stream_cmd = "@ADS" + std::to_string(b_on_off ? 1 : 0)
                         + "...........";

  serial_port_.sendCommand(stream_cmd); 

  if (true == b_on_off)
  {
    stream_flag_bits_ |= 0x02;  // Signaling streaming active.
  }
  else
  {
    stream_flag_bits_ &= 0xFFFD;  // Signaling streaming not active.
  }

  return;
}

void CppDriver::switchAnaStream(bool b_on_off)
{
  std::string stream_cmd = "@ADA" + std::to_string(b_on_off ? 1 : 0)
                         + "...........";

  serial_port_.sendCommand(stream_cmd); 

  if (true == b_on_off)
  {
    stream_flag_bits_ |= 0x08;  // Signaling streaming active.
  }
  else
  {
    stream_flag_bits_ &= 0xFFF7;  // Signaling streaming not active.
  }

  return;
}

void CppDriver::switchCurStream(bool b_on_off)
{
  std::string stream_cmd = "@ADC" + std::to_string(b_on_off ? 1 : 0)
                         + "...........";

  serial_port_.sendCommand(stream_cmd); 

  if (true == b_on_off)
  {
    stream_flag_bits_ |= 0x04;  // Signaling streaming active.
  }
  else
  {
    stream_flag_bits_ &= 0xFFFB;  // Signaling streaming not active.
  }

  return;
}

std::string CppDriver::numToStr(int16_t num, int8_t n_digits)
{
  char num_ascii[n_digits];

  for (int8_t ascii_it = n_digits - 1; ascii_it > -1; --ascii_it)
  {
    num_ascii[ascii_it] = num % 10 + 48;
    num /= 10;
  }

  std::string num_str(num_ascii, n_digits);

  return num_str;
}

void CppDriver::pollSerialPort()
{
  while (serial_trd_on_)
  {
    serial_port_.parseStream(thumb_info_, index_info_, mrl_info_,
                             is_checking_on_);
  }

  return;
}

void CppDriver::checkConnection()
{
  while (connection_trd_on_)
  {
    connection_mtx_.lock();
    is_checking_on_ = true;
    connection_mtx_.unlock();

    serial_port_.sendCommand("@?AreYouPlugged?");

    /* For each streaming that was activated, sending again the command for
     * activating it. This is done for keeping the streamings active in case of
     * a hand undesired restart during operation (e.g., after an excessive
     * undervoltage).
     */
    if (0x00 != (stream_flag_bits_ & 0x01))  // Motor positions streaming on
    {
      serial_port_.sendCommand("@ADP1...........");
    }

    if (0x00 != (stream_flag_bits_ & 0x02))  // Motor speeds streaming on
    {
      serial_port_.sendCommand("@ADS1...........");
    }

    if (0x00 != (stream_flag_bits_ & 0x04))  // Motor currents streaming on
    {
      serial_port_.sendCommand("@ADC1...........");
    }

    if (0x00 != (stream_flag_bits_ & 0x08))  // ADC streaming on
    {
      serial_port_.sendCommand("@ADA1...........");
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }

  return;
}
}  // namespace
