import serial
import time
import binascii
ser = serial.Serial(port="/dev/ttyUSB0",baudrate=115200,timeout=1,
parity=serial.PARITY_NONE,stopbits=serial.STOPBITS_ONE,bytesize=serial.EIGHTBITS)
counter = 0
while counter < 1:

   counter = counter + 1
   ser.write("\x09\x10\x03\xE8\x00\x03\x06\x00\x00\x00\x00\x00\x00\x73\x30")
   data_raw = ser.readline()
   #print(data_raw)
   data = binascii.hexlify(data_raw)
   print "Response 1 ", data
   time.sleep(0.01)
   ser.write("\x09\x03\x07\xD0\x00\x01\x85\xCF")
   data_raw = ser.readline()
  # print(data_raw)
   data = binascii.hexlify(data_raw)
   print "Response 2 ", data
   time.sleep(1)

 

#while(True):
   ''''
   ser.write("\x09\x03\x07\xD0\x00\x03\x04\x0E")
   data_raw = ser.readline()
   data = binascii.hexlify(data_raw)
   status = data[12:14]
   print "data ", data
   print "status is: ", status
   if status == "00":
       print "gripper is closed"
   elif status == "ff":
       print "gripper is open"
   else:
       ser.write("\x09\x10\x03\xE8\x00\x03\x06\x09\x00\x00\x00\xFF\xFF\x72\x19")
       data_raw = ser.readline()
       time.sleep(2)
   '''

   print "Close gripper"
   ser.write("\x09\x10\x03\xE8\x00\x03\x06\x09\x00\x00\xFF\xFF\xFF\x42\x29")
   data_raw = ser.readline()
   print(data_raw)
   data = binascii.hexlify(data_raw)
   print "Response 3 ", data
   time.sleep(2)
   
   ser.write("\x09\x03\x07\xD0\x00\x03\x04\x0E")
   data_raw = ser.readline()
  # print(data_raw)
   data = binascii.hexlify(data_raw)
   print "position ", data,data[6]
   print data[12:14]
   time.sleep(2)
	
   print "Open gripper"
   ser.write("\x09\x10\x03\xE8\x00\x03\x06\x09\x00\x00\x00\xFF\xFF\x72\x19")
   data_raw = ser.readline()

  # print(data_raw)
   data = binascii.hexlify(data_raw)
   print "Response 4 ", data
   time.sleep(2)

   ser.write("\x09\x03\x07\xD0\x00\x03\x04\x0E")
   data_raw = ser.readline()
   data = binascii.hexlify(data_raw)
   print "position ", data
   print data[12:14]
   time.sleep(2)


'''
if __name__ == '__main__':
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect(('10.10.1.101',30002))
    sock.send ('rq_reset()'+ '\n')
    time.sleep(0.1)
    sock.send ('rq_activate()'+ '\n')
    time.sleep(1)
    sock.send ('rq_open_and_wait()'+ '\n')
    time.sleep(1)
    sock.close()

'''
