#!/usr/bin/env python
#disable ssl-warnings
import urllib3
urllib3.disable_warnings()

import playsound
from gtts import gTTS

def main():
    tts = gTTS('hello hello hello')
    tts.save('hello.mp3')
    playsound.playsound('hello.mp3')

if __name__ == '__main__':
    main()
