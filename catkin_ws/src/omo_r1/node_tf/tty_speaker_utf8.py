# -*- coding: utf-8 -*-

import subprocess

def speak(text):

    subprocess.call(['espeak', '-ven+f3', '-k5', '-s150', text, '--stdout', '|', 'aplay', '-D', 'plughw:1,0'])

speak("안녕하세요. 이것은 espeak을 이용한 음성 출력 예제입니다.")
