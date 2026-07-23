/*
 ***********************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 ***********************************************************
 */

#ifndef __SOUND_PLAY__SOUND_PLAY__HPP__
#define __SOUND_PLAY__SOUND_PLAY__HPP__

#include <atomic>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sound_play_msgs/msg/sound_request.hpp>
#include <sound_play_msgs/action/sound_request.hpp>
#include <action_msgs/msg/goal_status.hpp>
#include <action_msgs/msg/goal_status_array.hpp>
#include <mutex>

namespace sound_play
{

/** \brief Class that sends goals to the sound_play node.
 *
 * This class is a helper class for communicating with the sound_play node
 * via the \ref sound_play::SoundRequest action. It has two ways of being used:
 *
 * - It can create Sound classes that represent a particular sound which
 *   can be played, repeated or stopped.
 *
 * - It provides methods for each way in which the sound_play::SoundRequest
 *   message can be invoked.
 */

class SoundClient
{
public:
  class Sound
  {
    friend class SoundClient;
  private:
    int snd_;
    float vol_;
    // @brief file name or text to say
    std::string arg_;
    // @brief other arguments
    std::string arg2_;
    SoundClient *client_;

    Sound(SoundClient *sc, int snd, const std::string &arg, const std::string arg2 = std::string(), const float vol = 1.0f)
    {
      client_ = sc;
      snd_ = snd;
      arg_ = arg;
      arg2_ = arg2;
      vol_ = vol;
    }

  public:
    /** \brief Play the Sound.
     *
     * This method causes the Sound to be played once.
     */
    void play()
    {
      client_->sendMsg(snd_, sound_play_msgs::msg::SoundRequest::PLAY_ONCE, arg_, arg2_, vol_);
    }

    /** \brief Play the Sound repeatedly.
     *
     * This method causes the Sound to be played repeatedly until stop() is
     * called.
     */
    void repeat()
    {
      client_->sendMsg(snd_, sound_play_msgs::msg::SoundRequest::PLAY_START, arg_, arg2_, vol_);
    }

    /** \brief Stop Sound playback.
     *
     * This method causes the Sound to stop playing.
     */
    void stop()
    {
      client_->sendMsg(snd_, sound_play_msgs::msg::SoundRequest::PLAY_STOP, arg_, arg2_, vol_);
    }
  };

  /** \brief Create a SoundClient that talks to the given action server
   *
   * Creates a SoundClient that sends goals to the sound_play action server
   * with the given name, relative to the given NodeHandle.
   *
   * \param nh Node handle to use when creating the action client.
   *
   * \param topic Name of the sound_play action to send goals to.
   */
  SoundClient(rclcpp::Node::SharedPtr nh, const std::string & topic)
  {
    init(nh, topic);
  }

  /** \brief Create a SoundClient with the default action name
   *
   * Creates a SoundClient that sends goals to the "sound_play" action server.
   *
   * \param nh Node handle to use when creating the action client.
   */
  SoundClient(rclcpp::Node::SharedPtr nh)
  {
    init(nh, "sound_play");
  }

  /** \brief Create a voice Sound.
   *
   * Creates a Sound corresponding to saying the indicated text.
   *
   * \param s Text to say
   * \param volume Volume at which to play the sound. 0 is mute, 1.0 is 100%.
   */
  Sound voiceSound(const std::string &s, float volume = 1.0f)
  {
    return Sound(this, sound_play_msgs::msg::SoundRequest::SAY, s, "", volume);
  }

  /** \brief Create a wave Sound.
   *
   * Creates a Sound corresponding to indicated file.
   *
   * \param s File to play. Should be an absolute path that exists on the
   * machine running the sound_play node.
   * \param volume Volume at which to play the sound. 0 is mute, 1.0 is 100%.
   */
  Sound waveSound(const std::string &s, float volume = 1.0f)
  {
    return Sound(this, sound_play_msgs::msg::SoundRequest::PLAY_FILE, s, "", volume);
  }

  /** \brief Create a wave Sound from a package.
   *
   * Creates a Sound corresponding to indicated file.
   *
   * \param p Package containing the sound file.
   * \param s Filename of the WAV or OGG file. Must be an path relative to the package valid
   * on the computer on which the sound_play node is running
   * \param volume Volume at which to play the sound. 0 is mute, 1.0 is 100%.
   */
  Sound waveSoundFromPkg(const std::string &p, const std::string &s, float volume = 1.0f)
  {
    return Sound(this, sound_play_msgs::msg::SoundRequest::PLAY_FILE, s, p, volume);
  }

  /** \brief Create a builtin Sound.
   *
   * Creates a Sound corresponding to indicated builtin wave.
   *
   * \param id Identifier of the sound to play.
   * \param volume Volume at which to play the sound. 0 is mute, 1.0 is 100%.
   */
  Sound builtinSound(int id, float volume = 1.0f)
  {
    return Sound(this, id, "", "", volume);
  }

  /** \brief Say a string
   *
   * Send a string to be said by the sound_node. The vocalization can be
   * stopped using stopSaying or stopAll.
   *
   * \param s String to say
   * \param volume Volume at which to play the sound. 0 is mute, 1.0 is 100%.
   */
  void say(
    const std::string &s, const std::string &voice = "voice_kal_diphone",
    float volume = 1.0f)
  {
    sendMsg(sound_play_msgs::msg::SoundRequest::SAY, sound_play_msgs::msg::SoundRequest::PLAY_ONCE, s, voice, volume);
  }

  /** \brief Say a string repeatedly
   *
   * The string is said repeatedly until stopSaying or stopAll is used.
   *
   * \param s String to say repeatedly
   * \param volume Volume at which to play the sound. 0 is mute, 1.0 is 100%.
   */
  void repeat(const std::string &s, float volume = 1.0f)
  {
    sendMsg(sound_play_msgs::msg::SoundRequest::SAY, sound_play_msgs::msg::SoundRequest::PLAY_START, s, "", volume);
  }

  /** \brief Stop saying a string
   *
   * Stops saying a string that was previously started by say or repeat. The
   * argument indicates which string to stop saying.
   *
   * \param s Same string as in the say or repeat command
   */
  void stopSaying(const std::string &s)
  {
    sendMsg(sound_play_msgs::msg::SoundRequest::SAY, sound_play_msgs::msg::SoundRequest::PLAY_STOP, s, "");
  }

  /** \brief Plays a WAV or OGG file
   *
   * Plays a WAV or OGG file once. The playback can be stopped by stopWave or
   * stopAll.
   *
   * \param s Filename of the WAV or OGG file. Must be an absolute path valid
   * on the computer on which the sound_play node is running
   * \param volume Volume at which to play the sound. 0 is mute, 1.0 is 100%.
   */
  void playWave(const std::string &s, float volume = 1.0f)
  {
    sendMsg(sound_play_msgs::msg::SoundRequest::PLAY_FILE, sound_play_msgs::msg::SoundRequest::PLAY_ONCE, s, "", volume);
  }

  /** \brief Plays a WAV or OGG file repeatedly
   *
   * Plays a WAV or OGG file repeatedly until stopWave or stopAll is used.
   *
   * \param s Filename of the WAV or OGG file. Must be an absolute path valid
   * on the computer on which the sound_play node is running.
   * \param volume Volume at which to play the sound. 0 is mute, 1.0 is 100%.
   */
  void startWave(const std::string &s, float volume = 1.0f)
  {
    sendMsg(sound_play_msgs::msg::SoundRequest::PLAY_FILE, sound_play_msgs::msg::SoundRequest::PLAY_START, s, "", volume);
  }

  /** \brief Stop playing a WAV or OGG file
   *
   * Stops playing a file that was previously started by playWave or
   * startWave.
   *
   * \param s Same string as in the playWave or startWave command
   */
  void stopWave(const std::string &s)
  {
    sendMsg(sound_play_msgs::msg::SoundRequest::PLAY_FILE, sound_play_msgs::msg::SoundRequest::PLAY_STOP, s);
  }

  /** \brief Plays a WAV or OGG file from a package
   *
   * Plays a WAV or OGG file once. The playback can be stopped by stopWaveFromPkg or
   * stopAll.
   *
   * \param p Package name containing the sound file.
   * \param s Filename of the WAV or OGG file. Must be an path relative to the package valid
   * on the computer on which the sound_play node is running
   * \param volume Volume at which to play the sound. 0 is mute, 1.0 is 100%.
   */
  void playWaveFromPkg(const std::string &p, const std::string &s, float volume = 1.0f)
  {
    sendMsg(sound_play_msgs::msg::SoundRequest::PLAY_FILE, sound_play_msgs::msg::SoundRequest::PLAY_ONCE, s, p, volume);
  }

  /** \brief Plays a WAV or OGG file repeatedly
   *
   * Plays a WAV or OGG file repeatedly until stopWaveFromPkg or stopAll is used.
   *
   * \param p Package name containing the sound file.
   * \param s Filename of the WAV or OGG file. Must be an path relative to the package valid
   * on the computer on which the sound_play node is running
   * \param volume Volume at which to play the sound. 0 is mute, 1.0 is 100%.
   */
  void startWaveFromPkg(const std::string &p, const std::string &s, float volume = 1.0f)
  {
    sendMsg(sound_play_msgs::msg::SoundRequest::PLAY_FILE, sound_play_msgs::msg::SoundRequest::PLAY_START, s, p, volume);
  }

  /** \brief Stop playing a WAV or OGG file
   *
   * Stops playing a file that was previously started by playWaveFromPkg or
   * startWaveFromPkg.
   *
   * \param p Package name containing the sound file.
   * \param s Filename of the WAV or OGG file. Must be an path relative to the package valid
   * on the computer on which the sound_play node is running
   */
  void stopWaveFromPkg(const std::string &p, const std::string &s)
  {
    sendMsg(sound_play_msgs::msg::SoundRequest::PLAY_FILE, sound_play_msgs::msg::SoundRequest::PLAY_STOP, s, p);
  }

  /** \brief Play a buildin sound
   *
   * Starts playing one of the built-in sounds. built-ing sounds are documented
   * in \ref SoundRequest.msg. Playback can be stopped by stopAll.
   *
   * \param sound Identifier of the sound to play.
   * \param volume Volume at which to play the sound. 0 is mute, 1.0 is 100%.
   */
  void play(int sound, float volume = 1.0f)
  {
    sendMsg(sound, sound_play_msgs::msg::SoundRequest::PLAY_ONCE, "", "", volume);
  }

  /** \brief Play a buildin sound repeatedly
   *
   * Starts playing one of the built-in sounds repeatedly until stop or stopAll 
   * is used. Built-in sounds are documented in \ref SoundRequest.msg.
   *
   * \param sound Identifier of the sound to play.
   * \param volume Volume at which to play the sound. 0 is mute, 1.0 is 100%.
   */
  void start(int sound, float volume = 1.0f)
  {
    sendMsg(sound, sound_play_msgs::msg::SoundRequest::PLAY_START, "", "", volume);
  }

  /** \brief Stop playing a built-in sound
   *
   * Stops playing a built-in sound started with play or start.
   *
   * \param sound Same sound that was used to start playback.
   */
  void stop(int sound)
  {
    sendMsg(sound, sound_play_msgs::msg::SoundRequest::PLAY_STOP);
  }

  /** \brief Stop all currently playing sounds
   *
   * Stops all speech, wave file, and built-in sound playback. It
   * requests cancellation of every goal that is still in flight and sends an
   * explicit ALL/PLAY_STOP request so that looping sounds are stopped and the
   * server's playback cache is cleared. Whether an already-playing one-shot
   * sound is interrupted immediately depends on the sound_play server honouring
   * the cancellation / processing the stop request while it is playing.
   */
  void stopAll()
  {
    // Request cancellation of any in-flight goals so the server can preempt an
    // actively playing sound.
    action_client_->async_cancel_all_goals();
    // Send an explicit ALL/PLAY_STOP request so looping sounds are stopped too.
    stop(sound_play_msgs::msg::SoundRequest::ALL);
  }

  /** \brief Check whether a sound is currently being played.
   *
   * Reflects the latest status published by the sound_play action server: it
   * returns true while a goal issued through this client is accepted or
   * executing (i.e. a sound is playing), and false otherwise. The status is
   * refreshed as the node is spun, so the node handle passed to the
   * constructor must be spun for this value to stay current.
   *
   * \return True if a sound is currently being played, false otherwise.
   */
  [[nodiscard]] bool isPlaying() const
  {
    return playing_;
  }

  /** \brief Turns warning messages on or off.
   *
   * If a goal is sent when the sound_play action server is not available, a
   * warning message is printed. This method can be used to enable or
   * disable warnings.
   *
   * \param state True to turn off messages, false to turn them on.
   */
  void setQuiet(bool state)
  {
    quiet_ = state;
  }

private:
  using SoundRequestAction = sound_play_msgs::action::SoundRequest;

  void init(rclcpp::Node::SharedPtr nh, const std::string &action_name)
  {
    nh_ = nh;
    action_client_ = rclcpp_action::create_client<SoundRequestAction>(nh, action_name);
    // Track the action server's goal status so callers can query playback
    // state via isPlaying(). This matches the QoS of the action status topic.
    status_sub_ = nh->create_subscription<action_msgs::msg::GoalStatusArray>(
      action_name + "/_action/status",
      rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
      [this](const action_msgs::msg::GoalStatusArray::SharedPtr msg) {
        updateStatus(*msg);
      });
    quiet_ = false;
  }

  void sendMsg(int snd, int cmd, const std::string &s = "", const std::string &arg2 = "", const float &vol = 1.0f)
  {
    sound_play_msgs::msg::SoundRequest msg;
    msg.sound = snd;
    msg.command = cmd;
    msg.arg = s;
    msg.arg2 = arg2;

    // ensure volume is in the correct range
    if (vol < 0) {
      msg.volume = 0;
    } else if (vol > 1.0) {
      msg.volume = 1.0f;
    } else {
      msg.volume = vol;
    }

    // TODO: Add this to diagnostics
    if (!action_client_->action_server_is_ready() && !quiet_) {
      RCLCPP_WARN( nh_->get_logger(), "Sound command issued, but the sound_play action server is not available. Perhaps you forgot to run soundplay_node.py");
    }

    // Fire-and-forget: deliver the goal to the action server without blocking.
    // The result is intentionally not awaited so the public API stays non-blocking.
    SoundRequestAction::Goal goal;
    goal.sound_request = msg;
    action_client_->async_send_goal(goal);
  }

  void updateStatus(const action_msgs::msg::GoalStatusArray &msg)
  {
    for (const auto &status : msg.status_list) {
      if (status.status == action_msgs::msg::GoalStatus::STATUS_ACCEPTED ||
        status.status == action_msgs::msg::GoalStatus::STATUS_EXECUTING)
      {
        playing_ = true;
        return;
      }
    }
    playing_ = false;
  }

  std::atomic<bool> quiet_{false};
  std::atomic<bool> playing_{false};
  rclcpp::Node::SharedPtr nh_;
  rclcpp_action::Client<SoundRequestAction>::SharedPtr action_client_;
  rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr status_sub_;
};

typedef SoundClient::Sound Sound;

}  // namespace sound_play

#endif
