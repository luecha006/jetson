import sys
from playsound import playsound

def displaysound_mp3(temp, maskPattern, TEMP_THRESHOLD):
    print('func sound')
    print('maskPattern ', maskPattern, ',temp ',temp)
    
    if  (temp > TEMP_THRESHOLD):
        #อุณหภูมิสูงเกิน
        playsound('sound/not_pass.mp3')
        playsound('sound/high_temperature.mp3')
        
    elif (temp <= TEMP_THRESHOLD) and (maskPattern == 'w'):
        #ผ่าน
        playsound('sound/pass.mp3')
        
    elif (temp <= TEMP_THRESHOLD) and (maskPattern == 'o'):
        #ไม่ผ่าน
        playsound('sound/not_pass.mp3')
        playsound('sound/please_wear_mask.mp3')
        
    elif  (temp <= TEMP_THRESHOLD) and (maskPattern == 'm'):
        #กรุณาสวมหน้ากาก
        playsound('sound/not_pass.mp3')
        playsound('sound/Cover_mouth_and_nose.mp3')

