#!/usr/bin/env python
# -*- coding: utf-8 -*-
import wave
import pyaudio
import rospy
from wav_service.srv import SetVoice
from wav_service.msg import Voice
import roslib.packages

WAV_PATH = roslib.packages.get_pkg_dir('wav_service') + '/wav/'

# FILENAME_MEN_="/home/robot/catkin_ws/src/RoboKen2023/wav_service/scripts/carenginestart1.wav"
# FILENAME_DOU = "/home/robot/catkin_ws/src/RoboKen2023/wav_service/scripts/musicbox.wav"
# FILENAME_KOTE = "/home/robot/catkin_ws/src/RoboKen2023/wav_service/scripts/strings.wav"

CHUNK = 1024


def wavrun(request):

    # yoroshiku
    if request.voice.id== Voice.YOROSHIKU:
        filename = WAV_PATH + "yoroshiku.wav"
    # dou
    elif request.voice.id== Voice.DOU:
        filename = WAV_PATH + "dou.wav"
    # kote
    elif request.voice.id== Voice.KOTE:
        filename = WAV_PATH + "kote.wav"
    # men
    elif request.voice.id == Voice.MEN:
        filename = WAV_PATH + "men.wav"
    #low_battery
    elif request.voice.id == Voice.LOW_BATTERY:
        filename = WAV_PATH + "low_battery.wav"
    #auto
    elif request.voice.id == Voice.AUTO:
        filename = WAV_PATH + "auto.wav"
    #manual
    elif request.voice.id == Voice.MANUAL:
        filename = WAV_PATH + "manual.wav"
    #tsuki
    elif request.voice.id == Voice.TSUKI:
        filename = WAV_PATH + "tsuki.wav"
    #arigto
    elif request.voice.id == Voice.ARIGATO:
        filename = WAV_PATH + "arigato.wav"

    
    w = wave.open(filename, 'rb')
    p = pyaudio.PyAudio()
    #ros info
    rospy.loginfo("play wav file")
    #ros info stream parameter
    rospy.loginfo("format: %s", p.get_format_from_width(w.getsampwidth()))
    rospy.loginfo("channels: %s", w.getnchannels())
    rospy.loginfo("rate: %s", w.getframerate())
    stream = p.open(format=p.get_format_from_width(w.getsampwidth()),
                channels=w.getnchannels(),
                rate=44100,
                output=True,
                output_device_index=11)

    data = w.readframes(CHUNK)
    while len(data) > 0:
        stream.write(data)
        data = w.readframes(CHUNK)

    stream.stop_stream()
    stream.close()
    p.terminate()

def voice_server():
    rospy.init_node('voice_node')   #ノードを初期化

    service = rospy.Service('wav_service', SetVoice, wavrun)   #'word_count'という名前で．型がWordCount，callback関数がcount_wordsのサーバーを公開
    #リクエスト受付の準備完了
    print("Ready voice server.")
    rospy.spin()

if __name__ == "__main__":
    voice_server()


