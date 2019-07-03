#############################################################################################################
#												IMPORTS														#
#############################################################################################################
import micropython, pyb, math,time,PID,utime
from pyb import ExtInt, DAC, Pin, UART,Switch
micropython.alloc_emergency_exception_buf(100)

#############################################################################################################
#											Initialise Variables											#
#############################################################################################################
# ----- zeros & int------#
ct=0
dacL,dacR=0,0
A_speed,B_speed=0,0
count,A_count,B_count=0,0,0
freq_L,freq_R = 0,0 #measured frequency
dr,dl,dc,x,y,phi=0,0,0,0,0,0
right_dist,left_dist,block_dist=0,0,0
flag,flag2,flag3,flaga,flagb=False,False,False,False,False
status,brake_status=False,False #if brake is applied this will be true otherwise it will be false
leftflag,rightflag,flagspeed,stopflag,startflag=2,2,2,2,2
l,r=33,10.16
total_dist=80
init_dacA,init_dacB=1630,1600
l_m_time_L1,l_m_time_R1 = 0,0 #last measured time (for calculating frequency)
l_m_time_L2,l_m_time_R2 = utime.ticks_us(),utime.ticks_us()#last measured time (for calculating frequency)
# ------- string--------#
s=""
state='front'
control_dict={"__START__":  1000,"__SPEED15__": 1001,"__SPEED20__": 1002,"__SPEED25__": 1003,"__STATUSUPDATE__": 1004,"__TURNRIGHT__": 2500,"__TURNLEFT__": 1500,"__BREAK__":  4000,"__SET__": 6000,"__RESETPIBOARD__" :5000} #Here __RESETPIBOARD__ is to stop the vehicle (ie., to set DAC=0)
# ------- Complex--------#
tim = pyb.Timer(10,freq=10)										#timer
dacA,dacB=DAC(1,bits=12),DAC(2,bits=12)							#DAC Initialization
pidL=PID.PID(45,0,90)											#PID
pidR=PID.PID(40,0,80)
# -------- PINS---------#
#Y1,Y2,Y3 are for ExtInt
motor_relay_L = Pin('Y4', Pin.OUT_PP)					#relay for brakes; connected in a normally open connection
motor_relay_R = Pin('Y5', Pin.OUT_PP)	
motor1_switch = Pin('Y6', Pin.OUT_PP)					#Relay for DAC; connected in a normally open connection
motor2_switch = Pin('Y7', Pin.OUT_PP) 
motor_L_brake = Pin('X9', Pin.OUT_PP)					#regenerative braking
motor_R_brake = Pin('X10',Pin.OUT_PP)


#############################################################################################################
#													FUNCTIONS												#
#############################################################################################################

# ===================================================== setSpeedL,R ======================================================#
def setSpeedL(setspL):
	pidL.setPoint(setspL)
def setSpeedR(setspR):
	pidR.setPoint(setspR)

# ============================================== forward,stop,start,left,right =====================================================#
# -------------------------------------------------------- forward --------------------------------------------------------#
def forward(setsp):
	setSpeedR(setsp)
	setSpeedL(setsp)
	tim.init(freq=(setsp//90))	
# -------------------------------------------------------- stop --------------------------------------------------------#
def stop():
	global motor_L_brake, motor_R_brake, tim, freq_L, freq_R
	tim.callback(None)
	motor_L_brake.low()
	motor_R_brake.low()
	pidL.setPoint(0)
	pidR.setPoint(0)
	freq_L = 0
	freq_R = 0
# -------------------------------------------------------- start --------------------------------------------------------#
def start():
	motor1_switch.low()
	motor2_switch.low()
	forward(2000)
	while(block_dist<total_dist):
		print(block_dist)
	stop()
# -------------------------------------------------------- left,right --------------------------------------------------------#
def left():
	global flagspeed, leftflag, init_dacA, init_dacB, count,right_dist,left_dist,startflag
	flagspeed,startflag,leftflag=3,3,3
	left_dist,right_dist=0,0

	motor_relay_L.low()
	time.sleep(0.2)
	dacA.write(0)
	time.sleep(0.2)
	
	init_dacA=1700
	init_dacB=1650
	while(right_dist<11 and left_dist<11):
		dacB.write(init_dacB)
		dacA.write(init_dacA)
	dacA.write(0)
	dacB.write(0)
	time.sleep(0.2)

	motor_relay_L.high()

	dacA.write(0)
	dacB.write(0)
	time.sleep(0.2)

	count=0
	flagspeed,leftflag=2,2
	init_dacA,init_dacB=0,0
def right():
	global flagspeed, rightflag, init_dacA, init_dacB, count, left_dist,startflag,right_dist
	flagspeed,startflag,rightflag=3,3,3
	left_dist,right_dist=0,0

	motor_relay_R.low()
	time.sleep(0.2)
	dacB.write(0)
	time.sleep(0.2)

	init_dacA=1700
	init_dacB=1650
	while(left_dist<11 and right_dist<11): #turning for a count of 11 hall pulses
		dacA.write(init_dacA)
		dacB.write(init_dacB)
	dacA.write(init_dacA)
	dacB.write(0)
	time.sleep(0.2)

	motor_relay_R.high()

	dacA.write(0)
	dacB.write(0)
	time.sleep(0.2)

	count=0
	init_dacA,init_dacB=0,0
	flagspeed,rightflag=2,2
# ===================================================== speedCorrection ======================================================#
def speedCorrection(dummy):
	global dacL, dacR
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
	pyb.LED(4).toggle()

# ===================================================== cfreq_L,R ======================================================#
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
	pyb.LED(2).toggle()

# ===================================================== isr_speed_timer ======================================================#
def isr_speed_timer(dummy):
	'''interrupt service routine for self correction'''
	global init_dacA,init_dacB,A_count,A_speed,B_count,B_speed,count,flag,rightflag,leftflag,stopflag,block_dist
	A_speed=A_count
	B_speed=B_count
	A_count=0
	B_count=0
	count+=1
	flag=True
	if(count<300 and flagspeed==2):
		mul=100				#higher multiplication factor for inital time, to avoid jerks to the vehicle
	else:
		mul=10
	x=A_speed-B_speed			#correction factor
	if(rightflag==2 and leftflag==2 and stopflag==2 and startflag==2 and x!=0):		#flags to ensure the self correction happens only when vehicle is moving straight
		init_dacA=init_dacA-(x*mul)
		init_dacB=init_dacB+(x*mul)
		dacA.write(init_dacA)
		dacB.write(init_dacB)
		init_dacA = min(2500,max(2500,init_dacA))
		init_dacB = min(2500,max(2500,init_dacB))

# ===================================================== brake_intr ======================================================#
def brake_intr(line):
	'''brake function. cuts off supply to the DAC'''
	global brake_status,status
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

# ===================================================== isr_motorA,B(ExtInt, not currently used) ======================================================#
def isr_motorA(dummy):
	'''interrupt service routine for  hall sensor measurements'''
	global A_count,dr, r, flag2,flaga,left_dist, block_dist
	A_count+=1
	flaga=True
	dr+=((2*math.pi*r)/30)
	flag2=True
	left_dist+=1
	block_dist+=1
def isr_motorB(dummy):
	global B_count,dl,r,flag2,flagb
	global right_dist
	B_count=B_count+1
	flagb=True
	dl+=((2*math.pi*r)/30)
	flag2=True
	right_dist+=1
		
# ===================================================== setmap ======================================================#
def setmap():
	global s, state, total_dist, ct
	s=s.rstrip('-')
	data = s.split("-")
	l=len(data)
	print(l)
	total_dist=160*l
	x0=data[0][0]
	y0=data[0][1]
	for i in range(1,l,1):
		x=data[i][0]
		y=data[i][1]
		if(state=='front'):
			if(y>y0 and x==x0):
				ct+=1
				start()
				state='front'	
			elif(x<x0 and y==y0):
				stop()
				ct=1
				total_dist=40
				left()
				stop()
				start()
				state='left'
			elif(x>x0 and y==y0):
				stop()
				ct=1
				total_dist=40
				right()
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
				left()
				stop()
				start()
				state='front'
			elif(y<y0 and x==x0):
				stop()
				ct=1
				total_dist=40
				right()
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
				right()
				stop()
				start()
				state='front'
			elif(y<y0 and x==x0):
				stop()
				ct=1
				total_dist=40
				left()
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
				left()
				stop()
				start()
				state='right'
			elif(y==y0 and x<x0):
				stop()
				ct=1
				total_dist=40
				right()
				stop()
				start()
				state='left'  
		x0=x
		y0=y 
	stop()
	print("map has been traced")             

#############################################################################################################
#														SETUP												#
#############################################################################################################
uart = UART(4, 9600)							#UART for ESP communication
uart.init(9600, bits=8, parity=None, stop=1,read_buf_len=64)
dacA.write(init_dacA)
dacB.write(init_dacB)
motor_relay_L.high()
motor_relay_R.high()
motor1_switch.high()
motor2_switch.high()
motor_L_brake.high()
motor_R_brake.high()

setSpeedR(0)
setSpeedL(0)
Switch().callback(lambda: forward(2000))
tim.callback(speedCorrection)
ExtInt('Y1',ExtInt.IRQ_RISING_FALLING,Pin.PULL_NONE,cfreq_R)#hall sensors as inperrupts
ExtInt('Y2',ExtInt.IRQ_RISING_FALLING,Pin.PULL_NONE,cfreq_L)
ExtInt(Pin('Y3'), ExtInt.IRQ_RISING, Pin.PULL_DOWN, brake_intr)# external intrupt to have sudden brake 
pyb.Timer(4,freq=33.33).callback(isr_speed_timer)

#############################################################################################################
#														LOOP												#
#############################################################################################################
while True:
	if(brake_status and status):
		motor1_switch.high()
		motor2_switch.high()
		status=False
	if(uart.any()):		
		uart_input_text=uart.read()
		print(uart_input_text)	
		uart_input_text=uart_input_text[0:-1]	
		contr_str=uart_input_text.decode("utf-8")
		print(contr_str)
		if( int(contr_str[0:4])==control_dict["__START__"]):
			brake_status=False
			motor_L_brake.high()
			motor_R_brake.high()
			ct=1
			start()
		if( int(contr_str[0:4])==control_dict["__TURNLEFT__"]):   
			left() 
		if( int(contr_str[0:4])==control_dict["__TURNRIGHT__"]):
			right() 
		if(int(contr_str[0:4])==control_dict["__STATUSUPDATE__"]):      
			print("hello")
		if( int(contr_str[0:4])==control_dict["__SET__"]):
			data=contr_str.split('\r\n')
			d2=data[1]
			s=d2	
			setmap()
		if(int(contr_str[0:4])==control_dict["__RESETPIBOARD__"]):    #this is the stop button
			stop()