# ===================================================== import =========================================#
import micropython, pyb, math,time,PID 
micropython.alloc_emergency_exception_buf(100)
import pyb
from pyb import ExtInt, DAC, Pin, UART,Switch
import utime
#f=open("hallNEW.txt","a+")
# ===================================================== Initial values =========================================#
tim = pyb.Timer(10,freq=10)
init_dacA=1630
init_dacB=1600
A_count=0
B_count=0
A_speed=0
B_speed=0
s=""
uart1_ct=0
state='front'
total_dist=80
count=0
block_dist=0
dr,dl,dc,x,y,phi=0,0,0,0,0,0
right_dist,left_dist,block_dist=0,0,0
l=33
r=10.16
flg=False
flg2=False	
flg3=False
flga=False
flgb=False
ct=0
yaw=""
rightflg=2
flgspeed=2
leftflg=2
stopflg=2
startflg=2
dacR=0
dacL=0

uart = UART(4, 9600)							#UART for ESP communication
uart.init(9600, bits=8, parity=None, stop=1,read_buf_len=64)
#uart1 = UART(3, 9600)							#UART for ultrasonic module
#uart1.init(9600, bits=8, parity=None, stop=1,read_buf_len=64)
dacA=DAC(1,bits=12)							#DAC Initialization
dacA.write(init_dacA)
dacB=DAC(2,bits=12)
dacB.write(init_dacB)
motor1_switch = Pin('Y6', Pin.OUT_PP)					#Relay for DAC; connected in a normally open connection
motor2_switch = Pin('Y7', Pin.OUT_PP) 
motor1_switch.high()
motor2_switch.high()

#last measured time (for calculating frequency) (for left wheel)
l_m_time_L1 = 0
l_m_time_L2 = utime.ticks_us() 

#last measured time (for calculating frequency) (for right wheel)
l_m_time_R1 = 0
l_m_time_R2 = utime.ticks_us()

freq_L = 0 #measured frequency (left wheel)
freq_R = 0 #measured frequency (right wheel)

pidL=PID.PID(45,0,90)
pidR=PID.PID(40,0,80)

def setSpeedL(setspL):
	global pidL, tim
	pidL.setPoint(setspL)
	#tim.init(freq=(setspL*10/60))
	
def setSpeedR(setspR):
	global pidR
	pidR.setPoint(setspR)

setSpeedR(0)
setSpeedL(0)

def forward(setsp):
	global pidL,pidR, tim
	global motor_L_brake, motor_R_brake
	setSpeedR(setsp)
	setSpeedL(setsp)
	tim.init(freq=(setsp//90))
	
def speedCorrection(dummy):
	#print("Done")
	global pidL,pidR,freq_L,freq_R,motor_L,motor_R, dacL, dacR
	dacR += pidR.update(int(freq_R))
	if(dacR<1024):
		dacR=1024
	elif(dacR>4095):
		dacR=4095
	dacL += pidL.update(int(freq_L))
	if(dacL<1024):
		dacL=1024
	elif(dacL>4095):
		dacL=4095
	dacB.write(dacL)
	dacA.write(dacR)
	#print("after update left =",dacL,"and right=",dacR)
	pyb.LED(4).toggle()

def cfreq_L(self):
	'''calculates frequency from hall sensor left wheel '''
	global l_m_time_L1, l_m_time_L2, freq_L, block_dist
	l_m_time_L1 = l_m_time_L2
	l_m_time_L2 = utime.ticks_us()
	block_dist+=1
	try:
		freq_L = 60000000//(l_m_time_L2 - l_m_time_L1)
	except ZeroDivisionError:
		print("Left error input")
	#print("Frequency Left =",freq_L)
	pyb.LED(1).toggle()
	
def cfreq_R(self):
	''' calculates frequency from hall sensor right wheel '''
	try:
		global l_m_time_R1, l_m_time_R2, freq_R
		l_m_time_R1 = l_m_time_R2
		l_m_time_R2 = utime.ticks_us()
	except:
		print("right error")
	try:
		freq_R=60000000//(l_m_time_R2-l_m_time_R1)
	except ZeroDivisionError:
		print("Right error input")
	#print("Frequency Right =",freq_R)
	pyb.LED(2).toggle()

def stop():
	global motor_L_brake, motor_R_brake, tim, freq_L, freq_R
	tim.callback(None)
	motor_L_brake.low()
	motor_R_brake.low()
	pidL.setPoint(0)
	pidR.setPoint(0)
	freq_L = 0
	freq_R = 0

sw = Switch()
#sw.callback(lambda: print("Frequency L =",freq_L,"rpm and Frequency R =",freq_R,"rpm"))
sw.callback(lambda: forward(2000))
tim.callback(speedCorrection)
#tim1.callback(freq)

#forward(4000)

#hall sensors as inperrupts
hall_R = ExtInt('Y1',ExtInt.IRQ_RISING_FALLING,Pin.PULL_NONE,cfreq_R)
hall_L = ExtInt('Y2',ExtInt.IRQ_RISING_FALLING,Pin.PULL_NONE,cfreq_L)	

tim = pyb.Timer(10,freq=10)
#f.write("A AND B VARYING SPEED")
#f.write("Speed-A"+"\t"+"DAC-A"+"\t"+"Speed-B"+"\t"+"DAC-B"+"\n")

'''  the two relays are connected to Y4 and Y5 for electronic brake control of both the bldc motor '''

motor_relay_L = Pin('Y4', Pin.OUT_PP)					#relay for brakes; connected in a normally open connection
motor_relay_R = Pin('Y5', Pin.OUT_PP)	
motor_relay_L.high()
motor_relay_R.high()


#regenerative braking
motor_L_brake = Pin('X9', Pin.OUT_PP)
motor_R_brake = Pin('X10', Pin.OUT_PP)
motor_L_brake.high()
motor_R_brake.high()



#interrupt service routine for  hall sensor measurements
def isr_motorA(dummy):
	global A_count
	global dr, r, flg2,flga,left_dist, block_dist
	A_count+=1
	flga=True
	dr+=((2*math.pi*r)/30)
	flg2=True
	left_dist+=1
	block_dist+=1
	#print(dr)
	
def isr_motorB(dummy):
    global B_count,dl,r,flg2,flgb
    global right_dist
    B_count=B_count+1
    flgb=True
    dl+=((2*math.pi*r)/30)
    flg2=True
    right_dist+=1
	#rint(dl)


#interrupt service routine for self correction
def isr_speed_timer(dummy):
	global init_dacA
	global init_dacB
	global A_count
	global A_speed
	global B_count
	global B_speed
	global count,flg,rightflg,leftflg,stopflg,block_dist
	A_speed=A_count
	B_speed=B_count
	A_count=0
	B_count=0
	count+=1
	flg=True
	if(count<300 and flgspeed==2):
		mul=100				#higher multiplication factor for inital time, to avoid jerks to the vehicle
	else:
		mul=10

	x=A_speed-B_speed			#correction factor
	if(rightflg==2 and leftflg==2 and stopflg==2 and startflg==2):		#flags to ensure the self correction happens only when behicle is moving straight
		#print("enter")
		
		#selfcorrection works on incrementing the DAC value for slower wheel and decrementing it for the faster one
                if(x>0):			
                        init_dacB=init_dacB+(x*mul)
                        dacB.write(init_dacB)
                        init_dacA=init_dacA-(x*mul)
                        dacA.write(init_dacA)
                        if(init_dacB>2500):
                                init_dacB=2500
                        if(init_dacA<1500):
                                init_dacA=1500

                elif(x<0):
                        init_dacB=init_dacB-(abs(x)*mul)
                        dacB.write(init_dacB)

                        init_dacA=init_dacA+(abs(x)*mul)
                        dacA.write(init_dacA)
                        if(init_dacA>2500):
                                init_dacA=2500
                        if(init_dacB<1500):
                                init_dacB=1500
	
		
	'''if(rightflg==3 or leftflg==3):
		print("enter_right_or_left")
                if(x>0):

                        init_dacB=init_dacB+(x*mul)
                        dacB.write(init_dacB)
                        init_dacA=init_dacA-(x*mul)
                        dacA.write(init_dacA)
                        if(init_dacB>2000):
                                init_dacB=2000
                        if(init_dacA<1650):
                                init_dacA=1650

                elif(x<0):
                        init_dacB=init_dacB-(abs(x)*mul)
                        dacB.write(init_dacB)
                        init_dacA=init_dacA+(abs(x)*mul)
                        dacA.write(init_dacA)
                        if(init_dacA>2000):
                                init_dacA=2000
                        if(init_dacB<1650):

                                init_dacB=1650'''
'''
#isr to read the data obtained from ultrasonic module
def isr_timer(dummy):
	if(uart1.any()):
		#print("hello")
		val=uart1.readchar()
		print("val",val)
		if(startflag==2 or rightflag==3 or leftflag==3):
			if(val==65):
				motor1_switch.high()
				motor2_switch.high()
				print("stoppeddd")

			elif(val==66):
				motor1_switch.low()
				motor2_switch.low()
				print("starteddd")
'''
#motorA1_int=ExtInt('Y1',ExtInt.IRQ_RISING_FALLING,Pin.PULL_NONE,isr_motorA)
#motorB1_int=ExtInt('Y2',ExtInt.IRQ_RISING_FALLING,Pin.PULL_NONE,isr_motorB)

#--------------------------------------------------------------------------------

brake_status=False #if brake is applied this will be true otherwise it will be false
status=False

#brake function. cuts off supply to the DAC
def brake_intr(line):
	global brake_status,status,brake_loop_status

	if(~(brake_status)):
		brake_status=True
		motor1_switch.high()
		motor2_switch.high()
		motor_L_brake.low()
		motor_R_brake.low()
		time.sleep(2)
		motor_L_brake.high()
		motor_R_brake.high()
		print("intrupt occured",line)
	#status=False

    
    

ext = ExtInt(Pin('Y3'), ExtInt.IRQ_RISING, Pin.PULL_DOWN, brake_intr)# external intrupt to have sudden brake 





#ext = ExtInt(Pin('Y3'), ExtInt.IRQ_RISING, Pin.PULL_DOWN, brake_intr)

#-----------------------------------------------------------------------------------

#bno_timer=pyb.Timer(3,freq=50)
#bno_timer.callback(isr_bno)

#timer initialization for ultrasonic and selfcorrection
#new_timer=pyb.Timer(3,freq=100)
#new_timer.callback(isr_timer)

speed_timer=pyb.Timer(4,freq=33.33)
speed_timer.callback(isr_speed_timer)


#control dictionary to convert received text to function using numbers
control_dict={"__START__":  1000,"__SPEED15__": 1001,"__SPEED20__": 1002,"__SPEED25__": 1003,"__STATUSUPDATE__": 1004,"__TURNRIGHT__": 2500,"__TURNLEFT__": 1500,
"__BREAK__":  4000,"__SET__": 6000,"__RESETPIBOARD__" :5000} #Here __RESETPIBOARD__ is to stop the vehicle (ie., to set DAC=0)  

'''def diff_right():
	print("right")
	global flgspeed, rightflg, init_dacA, init_dacB, count, left_dist,startflg,right_dist
	flgspeed=3
	startflg=3
	rightflg=3
	init_dacA=1700
	init_dacB=1650
	motor_relay_R.low()
	time.sleep(0.2)
	init_dacB=0
	dacB.write(init_dacB)
	time.sleep(0.2)
	init_dacB=1650
	left_dist=0
	right_dist=0
	while(left_dist<10 and right_dist<10):
		dacA.write(init_dacA)
		dacB.write(init_dacB)
	init_dacB=0
	dacA.write(init_dacA)
	dacB.write(init_dacB)
	time.sleep(0.2)
	motor_relay_R.high()
	init_dacB=0
	init_dacA=0
	dacA.write(init_dacA)
	dacB.write(init_dacB)
	count=0
	#time.sleep(3)
	rightflg=2
	flgspeed=2
	stop()
	start()'''


#differential right turn, works by reversing one wheel and turning for a cout of 11 hall pulses
def diff_right():
	global flgspeed, rightflg, init_dacA, init_dacB, count, left_dist,startflg,right_dist
	flgspeed=3
	startflg=3
	rightflg=3
	init_dacA=1700
	init_dacB=1650
	motor_relay_R.low()
	time.sleep(0.2)
	init_dacB=0
	dacB.write(init_dacB)
	time.sleep(0.2)
	init_dacB=1650
	left_dist=0
	right_dist=0
	while(left_dist<11 and right_dist<11):
		dacA.write(init_dacA)
		dacB.write(init_dacB)
	init_dacB=0
	dacA.write(init_dacA)
	dacB.write(init_dacB)
	time.sleep(0.2)
	motor_relay_R.high()
	init_dacB=0
	init_dacA=0
	dacA.write(init_dacA)
	dacB.write(init_dacB)
	count=0
	#time.sleep(3)
	rightflg=2
	flgspeed=2
	time.sleep(0.2)



def start():
	motor1_switch.low()
	motor2_switch.low()
	forward(2000)
	
	while(block_dist<total_dist):
		print(block_dist)
		pass
	stop()
	#print("start")
	'''global flgspeed,init_dacA,init_dacB,brake_status,status,count,stopflg,flg,flg3,startflg,block_dist,total_dist, ct
	block_dist=0
	if(~(brake_status)):
		if(ct==1):
			#print("start enter")
			motor1_switch.low()
			motor2_switch.low()
			init_dacA=1630
			init_dacB=1600
			dacA.write(init_dacA)
			dacB.write(init_dacB)
			flgspeed=2
			stopflg=2
			status=True
			count=0
			flg3=True
			ct+=1
		#if(ct>2):
			#total_dist=60
			
		
		while(block_dist<total_dist):
			startflg=2
			if(flgspeed==2 and total_dist<50):
				#print(count,flg,flg3)
				#------------------------------slow to high speed------------------------------------------------------------------
				if(count==30 and flg and flg3):
					init_dacA+=70
					init_dacB+=70
					flg=False
				if(count==60 and flg and flg3):
					init_dacA+=70
					init_dacB+=70
					flg=False
				if(count==90 and flg and flg3):
					init_dacA+=70
					init_dacB+=70
					flg=False
				if(count==120 and flg and flg3):
					init_dacA+=70
					init_dacB+=70
					flg=False
				if(count==150 and flg and flg3):
					init_dacA+=40
					init_dacB+=40
					flg=False
				if(count==180 and flg and flg3):
					init_dacA+=30
					init_dacB+=30
					flg=False
				if(count==210 and flg and flg3):
					init_dacA+=30
					init_dacB+=30
					flg=False
				if(count==240 and flg and flg3):

					init_dacA+=30
					init_dacB+=30
					flg=False
				dacA.write(init_dacA)
				dacB.write(init_dacB)
			'''
			    

'''def diff_left():
	print("left")
	global flgspeed, leftflg, init_dacA, init_dacB, count,right_dist,left_dist,startflg
	startflg=3
	flgspeed=3
	leftflg=3
	count=0
	init_dacB=1650
	init_dacA=1700
	motor_relay_L.low()
	time.sleep(0.2)
	init_dacA=0
	dacA.write(init_dacA)
	time.sleep(0.2)
	init_dacA=1700
	right_dist=0
	left_dist=0
	while(right_dist<9 and left_dist<9):
		dacB.write(init_dacB)
		dacA.write(init_dacA)
	init_dacA=0
	init_dacB=0
	dacB.write(init_dacB)
	dacA.write(init_dacA)
	time.sleep(0.2)
	motor_relay_L.high()
	init_dacA=0
	init_dacB=0
	dacA.write(init_dacA)
	dacB.write(init_dacB)
	count=0
	leftflg=2
	flgspeed=2
	stop()
	start()'''
	
'''def isr_bno(dummy):
	global yaw
	val=uart1.readline()    
    	yaw=val.decode("utf-8")
    	yaw=yaw.strip()    
    	print("yaw:",yaw)
    	uart.writechar(xy)'''

def diff_left():
	global flgspeed, leftflg, init_dacA, init_dacB, count,right_dist,left_dist,startflg
	startflg=3
	flgspeed=3
	#print("hellooo")
	leftflg=3
	count=0
	init_dacB=1650
	init_dacA=1700
	motor_relay_L.low()
	time.sleep(0.2)
	init_dacA=0
	dacA.write(init_dacA)
	time.sleep(0.2)
	init_dacA=1700
	right_dist=0
	left_dist=0
	while(right_dist<11 and left_dist<11):
		dacB.write(init_dacB)
		dacA.write(init_dacA)
	init_dacA=0
	init_dacB=0
	dacB.write(init_dacB)
	dacA.write(init_dacA)
	time.sleep(0.2)
	motor_relay_L.high()
	init_dacA=0
	init_dacB=0
	dacA.write(init_dacA)
	dacB.write(init_dacB)
	count=0
	leftflg=2
	flgspeed=2
	time.sleep(0.2)
	

"""	
def stop():
	#print("stop")
	global stopflg,A_speed,B_speed,init_dacA,init_dacB,block_dist
	stopflg=3
	if(A_speed<2000 and B_speed<2000):
		while(init_dacA>1400 or init_dacB>1400):
			init_dacA-=50
			init_dacB-=50
			dacA.write(init_dacA)
			dacB.write(init_dacB)
			time.sleep(0.02)
			#print("stopping")
	if(A_speed>2000 or B_speed>2000):
		while(init_dacA>1400 or init_dacB>1400):
			init_dacA-=20
			init_dacB-=20
			dacA.write(init_dacA)
			dacB.write(init_dacB)
			time.sleep(0.01)
			#print("stopping")

		
'''	motor_relay_R.low()
	motor_relay_L.low()
	time.sleep(0.2)
	init_dacA=0
	init_dacB=0
	dacA.write(init_dacA)			
	dacB.write(init_dacB)   ----to reverse stop
	time.sleep(0.2)
	init_dacA=2000			
	init_dacB=2000
	dacA.write(init_dacA)			
	dacB.write(init_dacB)
	time.sleep(0.1)
	motor_relay_R.high()
	motor_relay_L.high()
	time.sleep(0.02)
	init_dacA=0
	init_dacB=0
	dacA.write(init_dacA)			
	dacB.write(init_dacB)'''

	if(A_speed>B_speed):
		time.sleep(A_speed+2)
	else:
		time.sleep(B_speed+2)
"""
	
def setmap():
	#print("setmap")
        global s, state, total_dist, ct
        #print(s)
        s=s.rstrip('-')
        #print(s)
        data = s.split("-")
        #print(data)
        l=len(data)
        print(l)
        total_dist=160*l
        #print("l is",l)
        x0=data[0][0]
        y0=data[0][1]
        #print("x0 is",x0,"y0 is",y0)
        #print("present state is",state)
        for i in range(1,l,1):
                x=data[i][0]
                y=data[i][1]
                #print(x,y)
                #if(i=='\n\r'):
                        #break
                #print(x+" "+y)
                if(state=='front'):

			if(y>y0 and x==x0):
				ct+=1
				start()
				state='front'	
			elif(x<x0 and y==y0):
				stop()
				ct=1
				total_dist=40
				diff_left()
				stop()
				start()
				state='left'
			elif(x>x0 and y==y0):
				stop()
				ct=1
				total_dist=40
				diff_right()
				stop()
				start()
				state='right' 
		elif(state=='right'):	
			if(y==y0 and x>x0):
				ct+=1
				start()
				state='right'
			elif(y>y0 and x==x0):
				stop()
				ct=1
				total_dist=40
				diff_left()
				stop()
				start()

				state='front'
			elif(y<y0 and x==x0):
				stop()
				ct=1
				total_dist=40
				diff_right()
				stop()
				start()

				state='back'			
		elif(state=='left'):
			if(y==y0 and x<x0):
				ct+=1

				start()
				state='left'
			elif(y>y0 and x==x0):
				stop()
				ct=1
				total_dist=40
				diff_right()
				stop()
				start()

				state='front'
			elif(y<y0 and x==x0):
				stop()
				ct=1
				total_dist=40
				diff_left()
				stop()
				start()

				state='back'
		elif(state=='back'):	
			if(y<y0 and x==x0):
				ct+=1

				start()
				state='back'
			elif(y==y0 and x>x0):
				stop()
				ct=1
				total_dist=40
				diff_left()
				stop()
				start()

				state='right'
			elif(y==y0 and x<x0):
				stop()
				ct=1
				total_dist=40
				diff_right()
				stop()
				start()
				state='left'  
		#print("new state is",state) 
		x0=x
		y0=y 
	stop()
	print("map has been traced")             

while True:
	if(brake_status and status):
		#motor_relay_L.low()
		#motor_relay_R.low()
		motor1_switch.high()
		motor2_switch.high()
		status=False
		#print("brake_status")

   
	

	if(uart.any()):		
		#print(uart.any())
		#print(uart.read())
		uart_input_text=uart.read()
		
		#print("ok")
		print(uart_input_text)	
		uart_input_text=uart_input_text[0:-1]	
		contr_str=uart_input_text.decode("utf-8")
		#print(contr_str)

		print(contr_str)
		'''data=contr_str.split('\n')
		d1=data[0].trim()
		d2=data[1].trim()
		print("d1 is",d1)
		print("d2 is",d2)
		print(contr_str[0:4])'''
		#,contr_str[0:4])
		#print(contr_str[5:])

		#print(int(contr_str[0:4])==control_dict["__START__"])
		#control_dict={"__START__":  1000,"__SPEED15__": 1001,"__SPEED20__": 1002,"__SPEED25__": 1003,"__SPEED30__": 1004,"__TURNRIGHT__": 2500,"__TURNLEFT__": 1500,"__BREAK__":  4000,"__SETMAP__": 6000,"__RESETPIBOARD__" :5000} 

		if( int(contr_str[0:4])==control_dict["__START__"]):
			brake_status=False
			motor_L_brake.high()
			motor_R_brake.high()
		#	print("Hello")
			ct=1
			start()


		if( int(contr_str[0:4])==control_dict["__TURNLEFT__"]):   
			diff_left() 


		if( int(contr_str[0:4])==control_dict["__TURNRIGHT__"]):
			diff_right() 
		
		 
		if(int(contr_str[0:4])==control_dict["__STATUSUPDATE__"]):      
			print("hello")
			#uart.write(yaw)


		if( int(contr_str[0:4])==control_dict["__SET__"]):
           	#print(contr_str)
			data=contr_str.split('\r\n')
			#print(data)
			d2=data[1]
			#print("d1 is"d1,"done")
			#print("d2 is",d2,"done")
			s=d2	
			#print(s)
			setmap()

		if(int(contr_str[0:4])==control_dict["__RESETPIBOARD__"]):    #this is the stop button
			stop()	
			#f.close()
