#! /usr/bin/env python
import rospy
import wave
import pyaudio

from led_mode.srv import led_mode

filename="sample.wav"

CHUNK = 1024

def wavrun(request):
    w = wave.open(filename, 'rb')
    p = pyaudio.PyAudio()
    stream = p.open(format=p.get_format_from_width(w.getsampwidth()),
                channels=w.getnchannels(),
                rate=w.getframerate(),
                output=True)

    data = w.readframes(CHUNK)
    while len(data) > 0:
        stream.write(data)
        data = w.readframes(CHUNK)

    stream.stop_stream()
    stream.close()
    p.terminate()

rospy.init_node('service_server')   #ノードを初期化

service = rospy.Service('wav_service', led_mode, wavrun)   #'word_count'という名前で．型がWordCount，callback関数がcount_wordsのサーバーを公開
#リクエスト受付の準備完了

rospy.spin()
