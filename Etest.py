import serial, time

s = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
time.sleep(2)
s.reset_input_buffer()

def send(cmd, wait=30):
    print('>>', cmd)
    s.write((cmd + '\n').encode())
    end = time.time() + wait
    while time.time() < end:
        line = s.readline().decode(errors='ignore').strip()
        if line:
            print('   ', line)
        if line.startswith('ok'):
            break

for c in ['M115',          # 펌웨어 종류/버전
          'M105',          # 현재 온도
          'M302',          # 저온 압출 설정 상태
          'M302 S0',       # 저온 압출 허용
          'M302',          # 바뀌었는지 확인
          'T0', 'M83',
          'G1 E20 F300',   # 20mm 밀기 (약 4초)
          'M400']:
    send(c)

s.close()
