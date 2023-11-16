#!/usr/bin/env python
# -*- coding: utf-8 -*-
import wave
import pyaudio
import rospy
from wav_service.srv import Voice

# FILENAME_MEN_="/home/robot/catkin_ws/src/RoboKen2023/wav_service/scripts/carenginestart1.wav"
# FILENAME_DOU = "/home/robot/catkin_ws/src/RoboKen2023/wav_service/scripts/musicbox.wav"
# FILENAME_KOTE = "/home/robot/catkin_ws/src/RoboKen2023/wav_service/scripts/strings.wav"

CHUNK = 1024


def wavrun(request):

    if request.a == 1:
        filename = "/home/robot/catkin_ws/src/RoboKen2023/wav_service/scripts/carenginestart1.wav"
        print("wav1" + filename)
    elif request.a == 2:
        filename = "/home/robot/catkin_ws/src/RoboKen2023/wav_service/scripts/musicbox.wav"
        print("wav2" + filename)
    else :
        filename = "/home/robot/catkin_ws/src/RoboKen2023/wav_service/scripts/strings.wav"
        print("wav3" + filename)

    
    w = wave.open(filename, 'rb')
    p = pyaudio.PyAudio()
    stream = p.open(format=p.get_format_from_width(w.getsampwidth()),
                channels=w.getnchannels(),
                rate=w.getframerate(),
                output=True,
                output_device_index=11)

    data = w.readframes(CHUNK)
    while len(data) > 0:
        stream.write(data)
        data = w.readframes(CHUNK)

    stream.stop_stream()
    stream.close()
    p.terminate()
    # PyAudioインスタンスを作成
    #p = pyaudio.PyAudio()

    # # 利用可能なオーディオデバイスの数を取得
    # num_devices = p.get_device_count()

    # # デバイスの情報を表示
    # print("Available audio input devices:")
    # for i in range(num_devices):
    #     device_info = p.get_device_info_by_index(i)
    #     if device_info['maxInputChannels'] > 0:
    #         print("Device ID {}: {}".format(i, device_info['name']))


    # # PyAudioインスタンスを終了
    # p.terminate()


def voice_server():
    rospy.init_node('service_server')   #ノードを初期化

    service = rospy.Service('wav_service', Voice, wavrun)   #'word_count'という名前で．型がWordCount，callback関数がcount_wordsのサーバーを公開
    #リクエスト受付の準備完了
    print("Ready voice server.")
    rospy.spin()

if __name__ == "__main__":
    voice_server()


