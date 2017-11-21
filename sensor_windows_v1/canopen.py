#**************************************************************************
#
#            Copyright (c) 2004 - 2007 by esd electronic system design gmbh
#
#  This software is copyrighted by and is the sole property of 
#  esd gmbh.  All rights, title, ownership, or other interests
#  in the software remain the property of esd gmbh. This
#  software may only be used in accordance with the corresponding
#  license agreement.  Any unauthorized use, duplication, transmission,  
#  distribution, or disclosure of this software is expressly forbidden.
#
#  This Copyright notice may not be removed or modified without prior
#  written consent of esd gmbh.
#
#  esd gmbh, reserves the right to modify this software without notice.
#
#  electronic system design gmbh          Tel. +49-511-37298-0
#  Vahrenwalder Str 207                   Fax. +49-511-37298-68 
#  30165 Hannover                         http://www.esd-electronics.com
#  Germany                                sales@esd-electronics.com
#
#**************************************************************************
#
#     Module Name: canopen.py
#         Version: 1.00
#   Original Date: 28-Oct-2004
#          Author: Michael Schoppe
#        Language: python
# Compile Options: 
# Compile defines: 
#       Libraries: 
#    Link Options: 
#
#    Entry Points: 
#
#**************************************************************************
# Description
#
#                                                                        
#                                                                        
# Edit Date/Ver   Edit Description
# ==============  ===================================================
# OT   27/07/07   Removed additional __init__ in SYNC class which caused
#                  problems for Python 2.5
# MS   28/10/05   Written
# MS   14/03/05   add pdo.write method in class PDO to send data in one PDO
#                  with different lengths
# MS   01/06/05   add sdo.scan method in class SDO to scan object dictionary



import time 
import string
import ntcan  		#Importiere Wrapper fuer NTCAN.DLL
import threading


# Baudrates
CANOPEN_1000kBd = 0
CANOPEN_666kBd = 1
CANOPEN_500kBd = 2
CANOPEN_333kBd = 3
CANOPEN_250kBd = 4
CANOPEN_166kBd = 5
CANOPEN_125kBd = 6
CANOPEN_100kBd = 7
CANOPEN_66kBd = 8
CANOPEN_50kBd = 9
CANOPEN_33kBd = 10
CANOPEN_20kBd = 11
CANOPEN_12kBd = 12
CANOPEN_10kBd = 13

# Baudrates
Baudrate =\
{\
	CANOPEN_1000kBd : "1000 kBd",\
	CANOPEN_666kBd : "666.6 kBd",\
	CANOPEN_500kBd : "500 kBd",\
	CANOPEN_333kBd : "333.3 kBd",\
	CANOPEN_250kBd : "250 kBd",\
	CANOPEN_166kBd : "166.6 kBd",\
	CANOPEN_125kBd : "125 kBd",\
	CANOPEN_100kBd : "100 kBd",\
	CANOPEN_66kBd : "66.6 kBd",\
	CANOPEN_50kBd : "50 kBd",\
	CANOPEN_33kBd : "33 kBd",\
	CANOPEN_20kBd : "20 kBd",\
	CANOPEN_12kBd : "12.5 kBd",\
	CANOPEN_10kBd : "10 kBd",\
}\

# COB-IDs
NMT_ID = 0x000     		#    0 ->    0
SYNC_ID = 0x080     		#  128 ->  128
EMERGENCY_ID = 0x080    	#  128 ->  129 bis 255
TIMESTAMP_ID = 0x100    	#  256 ->  256
PDO1TX_ID = 0x180     	#  384 ->  385 bis 511
PDO1RX_ID = 0x200     	#  512 ->  513 bis 639
PDO2TX_ID = 0x280     	#  640 ->  641 bis 767
PDO2RX_ID = 0x300     	#  768 ->  769 bis 895
PDO3TX_ID = 0x380     	#  896 ->  897 bis 1023
PDO3RX_ID = 0x400     	# 1024 -> 1025 bis 1151
PDO4TX_ID = 0x480     	# 1152 -> 1153 bis 1279
PDO4RX_ID = 0x500     	# 1280 -> 1281 bis 1407
SDOTX_ID = 0x580     		# 1408 -> 1409 bis 1535
SDORX_ID = 0x600     		# 1536 -> 1537 bis 1663
NODEGUARD_ID = 0x700    	# 1792 -> 1793 bis 1919


# NMT -------------------------------------------------------------------------
NMT_STARTREMOTENODE = 0x01
NMT_STOPREMOTENODE =  0x02
NMT_ENTERPREOPERATIONAL = 0x80
NMT_RESETNODE = 0x81
NMT_RESETCOMMUNICATION = 0x82

NMTstate =\
{\
	NMT_STARTREMOTENODE : "Start Node",\
	NMT_STOPREMOTENODE : "Stop Node",\
	NMT_ENTERPREOPERATIONAL : "Enter Preoperational",\
	NMT_RESETNODE : "Reset Node",\
	NMT_RESETCOMMUNICATION : "Reset Communication"\
}\


# SYNC ------------------------------------------------------------------------




# EMERGENCY -------------------------------------------------------------------

EmergencyErrorCodes =\
{\
	0x0000 : "Error Reset or no Error",\
	0x0001 : "tbd",\
	0x0002 : "tbd"\
}\


# PDO -------------------------------------------------------------------------





# SDO -----------------------------------------------------------------------
# download
SDO_SEG_REQ_INIT_DOWNLOAD_xBYTE = 0x22
SDO_SEG_REQ_INIT_DOWNLOAD_1BYTE = 0x2F      
SDO_SEG_REQ_INIT_DOWNLOAD_2BYTE = 0x2B      
SDO_SEG_REQ_INIT_DOWNLOAD_3BYTE = 0x27      
SDO_SEG_REQ_INIT_DOWNLOAD_4BYTE = 0x23      
SDO_SEG_RES_INIT_DOWNLOAD = 0x60
# upload 
SDO_SEG_REQ_INIT_UPLOAD = 0x40
SDO_SEG_RES_INIT_UPLOAD_nBYTE =  0x41      
SDO_SEG_RES_INIT_UPLOAD_xBYTE =  0x42      
SDO_SEG_RES_INIT_UPLOAD_1BYTE = 0x4F      
SDO_SEG_RES_INIT_UPLOAD_2BYTE = 0x4B      
SDO_SEG_RES_INIT_UPLOAD_3BYTE = 0x47      
SDO_SEG_RES_INIT_UPLOAD_4BYTE = 0x43      
SDO_SEG_REQ_UPLOAD1 = 0x60
SDO_SEG_REQ_UPLOAD1 = 0x70


SDO_SEG_ABORT_TRANSFER =0x80

# error codes
SDOerror =\
{\
	0x00000000 : "Error Reset or no Error",\
	0x05030000 : "Toggle bit not alternated",\
	0x05040000 : "SDO protocol timed out",\
	0x05040001 : "Client/server command specifier not valid or unknown",\
	0x05040002 : "Invalid block size (block mode only)",\
	0x05040003 : "Invalid sequence number (block mode only)",\
	0x05040004 : "CRC error (block mode only)",\
	0x05040005 : "Out of memory",\
	0x06010000 : "Unsupported access to an object",\
	0x06010001 : "Attempt to read a write only object",\
	0x06010002 : "Attempt to write a read only object",\
	0x06020000 : "Object does not exist in the object dictionary",\
	0x06040041 : "Object cannot be mapped to the PDO",\
	0x06040042 : "The number and length of the objects to be mapped would exceed PDO length",\
	0x06040043 : "General parameter incompatibility reason",\
	0x06040047 : "General internal incompatibility in the device",\
	0x06060000 : "Access failed due to an hardware error",\
	0x06070010 : "Data type does not match, length of service parameter does not match",\
	0x06070012 : "Data type does not match, length of service parameter too high",\
	0x06070013 : "Data type does not match, length of service parameter too low",\
	0x06090011 : "Sub-index does not exist",\
	0x06090030 : "Value range of parameter exceeded (only for write access)",\
	0x06090031 : "Value of parameter written too high",\
	0x06090032 : "Value of parameter written too low",\
	0x06090036 : "Maximum value is less than minimum value",\
	0x08000000 : "general error",\
	0x08000020 : "Data cannot be transferred or stored to the application",\
	0x08000021 : "Data cannot be transferred or stored to the application because of local control",\
	0x08000022 : "Data cannot be transferred or stored to the application because of the present device state",\
	0x08000023 : "Object dictionary dynamic generation fails or no object dictionary is present (e.g. object dictionary is generated from file and generation fails because of an file error)"\
}\

# NODEGUARD -------------------------------------------------------------------
NODEGUARD_BOOTUP_STATE = 0
NODEGUARD_STOPPED_STATE = 4
NODEGUARD_OPERATIONAL_STATE = 5
NODEGUARD_PREOPERATIONAL_STATE = 127

NODEGUARDstate =\
{\
	NODEGUARD_BOOTUP_STATE : "Bootup-State",\
	NODEGUARD_STOPPED_STATE : "Stopped-State",\
	NODEGUARD_OPERATIONAL_STATE : "Operational-State",\
	NODEGUARD_PREOPERATIONAL_STATE : "PreOperational-State"\
}\



# Functions ------------------------------------------------------------------
# SYNC
def SYNCsend (cif):
	print "SYNCsend"
	cmsg = ntcan.CMSG()
	cmsg.canWriteByte(cif, SYNC_ID,0)
	del cmsg
	
class SYNC (threading.Thread):
	def __init__(self,net,baudrate,TxQS,TxTO,time_s=1.0,id=SYNC_ID):
		threading.Thread.__init__(self)
		self.cif = ntcan.CIF(net,TxQS,TxTO)
		self.cif.baudrate = baudrate
		self.cmsg = ntcan.CMSG()
		self.isrun=True
		self.time_s = time_s
		self.id=id
#	def __init__(self,net,baudrate,time_s=1.0,id=SYNC_ID):
#		threading.Thread.__init__(self)
#		self.cif = ntcan.CIF(net,1,2000)
#		self.cif.baudrate = baudrate
#		self.cmsg = ntcan.CMSG()
#		self.isrun=True
#		self.time_s = time_s
#		self.id=id
	def run (self):
		print "SYNC start %fs"%self.time_s
		while self.isrun:
			self.send()
			time.sleep(self.time_s)
		print "SYNC stopped"
	def send (self):
		print "SYNC send"
		self.cmsg.canWriteByte(self.cif,self.id,0)
	def suspend (self):
		print "SYNC suspend"
		self.isrun=False
	def resume (self):
		print "SYNC resume"
		self.isrun=True
		self.run()



# NMT Services
# node = 1..127 , 0=all nodes!
def NMTstart (cif,node):
	print "NMTstart node: %d " %node
	cmsg = ntcan.CMSG()
	cmsg.canWriteByte(cif, NMT_ID, 2, NMT_STARTREMOTENODE, node)
	del cmsg
	
def NMTstop (cif,node):
	print "NMTstop node: %d " %node
	cmsg = ntcan.CMSG()
	cmsg.canWriteByte(cif, NMT_ID, 2, NMT_STOPREMOTENODE, node)
	del cmsg

def NMTpreop (cif,node):
	print "NMTpreop node: %d " %node
	cmsg = ntcan.CMSG()
	cmsg.canWriteByte(cif, NMT_ID, 2, NMT_ENTERPREOPERATIONAL, node)
	del cmsg

def NMTreset (cif,node):
	print "NMTreset node: %d " %node
	cmsg = ntcan.CMSG()
	cmsg.canWriteByte(cif, NMT_ID, 2, NMT_RESETNODE, node)
	del cmsg

def NMTresetcomm (cif,node):
	print "NMTresetcomm node: %d " %node
	cmsg = ntcan.CMSG()
	cmsg.canWriteByte(cif, NMT_ID, 2, NMT_RESETCOMMUNICATION, node)
	del cmsg

# 
class NMT:
	def __init__(self,net,baudrate,TxQS,TxTO):
		self.cif = ntcan.CIF(net,TxQS,TxTO)
		self.cif.baudrate=baudrate
		self.cmsg = ntcan.CMSG()
		#del cmsg
	def start (self,node):
		print "NMTstart node: %d " %node
		self.cmsg.canWriteByte(self.cif, NMT_ID, 2, NMT_STARTREMOTENODE, node)
	def stop (self,node):
		print "NMTstop node: %d " %node
		self.cmsg.canWriteByte(self.cif, NMT_ID, 2, NMT_STOPREMOTENODE, node)
	def preop (self,node):
		print "NMTpreop node: %d " %node
		self.cmsg.canWriteByte(self.cif, NMT_ID, 2, NMT_ENTERPREOPERATIONAL, node)
	def reset (self,node):
		print "NMTreset node: %d " %node
		self.cmsg.canWriteByte(self.cif, NMT_ID, 2, NMT_RESETNODE, node)
	def resetcomm (self,node):
		print "NMTresetcomm node: %d " %node
		self.cmsg.canWriteByte(self.cif, NMT_ID, 2, NMT_RESETCOMMUNICATION, node)



# NODEGUARD -------------------------------------------------------------------------
def NODEGUARDrequest (cif,node,state=0,result=True):
	cmsg = ntcan.CMSG()
	print "Nodeguard RTR %d -> " % node,
	cif.canIdAdd(NODEGUARD_ID+node)
	cmsg.canWriteByte(cif, NODEGUARD_ID+node, 0x10) # rtr
	try: 
		cmsg.canRead(cif)
		state = cmsg.data.c[0]
#		print cmsg 
		if cmsg.len == 1:
			print NODEGUARDstate.get(cmsg.data.c[0] & 0x7F)
	except IOError, (errno):
		    print "I/O error(%s): " % (errno)
		    result=False
	cif.canIdDelete(NODEGUARD_ID+node)
	del cmsg
	return result,state

class HeartbeatProducer (threading.Thread):
	def __init__(self,net,baudrate,node,time_s,state=NODEGUARD_BOOTUP_STATE):
		threading.Thread.__init__(self)
		self.net=net					# logical CAN Network [0, 255]
		#self.RxQS=1			        		# RxQueueSize [0, 10000]
		#self.RxTO=2000					# RxTimeOut in Millisconds
		self.TxQS=1					# TxQueueSize [0, 10000]
		self.TxTO=1000				# TxTimeOut in Millseconds
		#self.cif = ntcan.CIF(self.net,self.RxQS,self.RxTO,self.TxQS,self.TxTO)
		self.node=node
		self.cif = ntcan.CIF(self.net,self.TxQS,self.TxTO)
		self.cif.baudrate=baudrate
		self.cmsg = ntcan.CMSG()
		self.time_s = time_s
		self.state = state
	def run (self):
		self.cmsg.canWriteByte(self.cif, (NODEGUARD_ID+self.node),1,self.state) 
		print "Heartbeat node %d: 0x%0.2X (first frame)" %(self.node,self.state)
		while 1:
			time.sleep(self.time_s)
			self.state=(self.state+0x80)&0xFF
			self.cmsg.canWriteByte(self.cif, (NODEGUARD_ID+self.node),1,self.state)
			print "Heartbeat node %d: 0x%0.2X" %(self.node,self.state)
	def changeState (self,state):
		lstate = self.state&0x80
		self.state = state&0x7F
		self.state = lstate|self.state
	def suspend (self):
		print "HeartbeatProducer suspend"
		self.isrun=False
	def resume (self):
		print "HeartbeatProducer resume"
		self.isrun=True
		self.run()


class HeartbeatConsumer (threading.Thread):
	def __init__(self,net,baudrate,node,time_s):
		threading.Thread.__init__(self)
		self.net=net					# logical CAN Network [0, 255]
		self.RxQS=1000		        		# RxQueueSize [0, 10000]
		self.RxTO=int(time_s*1000) 			# RxTimeOut in Millisconds
		self.TxQS=1					# TxQueueSize [0, 10000]
		self.TxTO=1000					# TxTimeOut in Millseconds
		self.cif = ntcan.CIF(self.net,self.RxQS,self.RxTO,self.TxQS,self.TxTO)
		self.cif.baudrate=baudrate
		self.cmsg = ntcan.CMSG()
		self.node=node
		self.time_s = time_s
		self.state = 0
		self.isrun=True
	def run (self):
		self.cif.canIdAdd(NODEGUARD_ID+self.node)
		while self.isrun==True:
			try: 
				self.cmsg.canRead(self.cif)
				self.state = self.cmsg.data.c[0]
				print "Heartbeat Consumer node: %d " %self.node,
				if self.cmsg.len == 1:
					print NODEGUARDstate.get(self.cmsg.data.c[0] & 0x7F)
				else: 
					print "Wrong cmsg.len: %d"%self.cmsg.len
			except IOError, (errno):
				print "Heartbeat Consumer node: %d -> I/O error(%s)" %(self.node,errno)
				#print "I/O error(%s, %d): " % (errno,errno)
		self.cif.canIdDelete(NODEGUARD_ID+self.node)
	def suspend (self):
		print "HeartbeatConsumer suspend"
		self.isrun=False
	def resume (self):
		print "HeartbeatConsumer resume"
		self.isrun=True
		self.run()
		


class NODEGUARD (threading.Thread):
	def __init__(self,net,baudrate,RxQS,RxTO,TxQS,TxTO,time_s=1.0):
		threading.Thread.__init__(self)
		self.cif = ntcan.CIF(net,RxQS,RxTO,TxQS,TxTO)
		self.cif.baudrate=baudrate
		self.cmsg = ntcan.CMSG()
		self.isrun=True
		self.time_s = time_s
	def run (self,node):
		print "NODEGUARD start %fs"%self.time_s
		while self.isrun:
			self.request(node)
			time.sleep(self.time_s)
		print "NODEGUARD stopped"
	def request (self,node,state=0,result=True):
		print "NODEGUARD RTR %d -> " % node,
		self.cif.canIdAdd(NODEGUARD_ID+node)
		self.cmsg.canWriteByte(self.cif, NODEGUARD_ID+node, 0x10) # rtr
		try: 
			self.cmsg.canRead(self.cif)
			state = self.cmsg.data.c[0]
			if self.cmsg.len == 1:
				print NODEGUARDstate.get(self.cmsg.data.c[0] & 0x7F)
			else:
				print "Wrong cmsg.len %d"%(self.cmsg.len)
		except IOError, (errno):
				print "I/O error(%s): " % (errno)
				result=False
		self.cif.canIdDelete(NODEGUARD_ID+node)
		return result,state
	def suspend (self):
		print "NODEGUARD suspend"
		self.isrun=False
	def resume (self):
		print "NODEGUARD resume"
		self.isrun=True
		self.run()

		
# EMERGENCY	
class EMERGENCY:
	def __init__(self,net,baudrate,RxQS,RxTO,TxQS,TxTO):
		self.cif = ntcan.CIF(net,RxQS,RxTO,TxQS,TxTO)
		self.cif.baudrate = baudrate
		self.cmsg = ntcan.CMSG()
		#del cmsg
	def monitor (self,node,emerg_error_code=0,error_reg=0,no_of_errors=0,manu_specific_error_field=0,result=True):
		self.cif.canIdAdd(EMERGENCY_ID+node)
		#print "EMERGENCYmonitor node: %d -> " %(node),
		try: 
			self.cmsg.canRead(self.cif)
			if self.cmsg.len == 8:
				result = True
				emerg_error_code=self.cmsg.data.s[0]
				error_reg=self.cmsg.data.c[2]
				no_of_errors=self.cmsg.data.c[3]
				manu_specific_error_field=self.cmsg.data.l[1]
				print "EMERGENCYmonitor node: %d -> " %(node),
				print "0x%0.4X  0x%0.2X  %d  0x%0.8X" %(emerg_error_code,error_reg,no_of_errors,manu_specific_error_field)
			else:
				result = False
				
		except IOError, (errno):
				#print "I/O error(%s): " % (errno)
				result = False
		self.cif.canIdDelete(EMERGENCY_ID+node)
		return result
		
		
		
# SDO -------------------------------------------------------------------------
def SDOread (cif,node,index,subindex,data=0,result=True):
	cmsg = ntcan.CMSG()
	rdindex = 0
	cif.canIdAdd(SDOTX_ID+node)
	print "SDOread node: %d index: 0x%0.4X subindex: 0x%0.2X -> " %(node,index,subindex),
	cmsg.canWriteByte(cif, SDORX_ID+node,8, SDO_SEG_REQ_INIT_UPLOAD,index & 0xFF,(index & 0xFF00)>>8,subindex,0,0,0,0)
	try: 
		cmsg.canRead(cif)
#		print cmsg 
		if cmsg.len == 8:
			rdindex = cmsg.data.c[1] + cmsg.data.c[2] *256
#			print "index: %x  %x  %x" % (rdindex,cmsg.data.c[1],cmsg.data.c[2]*256)
			if (rdindex == index and cmsg.data.c[3] == subindex):
				if cmsg.data.c[0] == SDO_SEG_ABORT_TRANSFER:
					print "error: 0x%0.8X %s" % (cmsg.data.l[1],SDOerror.get(cmsg.data.l[1]))
					result = False
					data = cmsg.data.l[1]
				elif cmsg.data.c[0] == SDO_SEG_RES_INIT_UPLOAD_xBYTE:      
					data = cmsg.data.l[1]
					print "data: %d  0x%0.8X" % (data,data)
				elif cmsg.data.c[0] == SDO_SEG_RES_INIT_UPLOAD_1BYTE:      
					data = cmsg.data.l[1] & 0xFF
					print "data: %d  0x%0.2X" % (data,data),
					if (cmsg.data.c[4] >= 0x20): print "%c" % cmsg.data.c[4]
					else: print " "
				elif cmsg.data.c[0] == SDO_SEG_RES_INIT_UPLOAD_2BYTE:      
					data = cmsg.data.l[1] & 0xFFFF
					print "data: %d  0x%0.4X" % (data,data),
					if (cmsg.data.c[4] >= 0x20): print "%c" % cmsg.data.c[4],
					else: print " ",
					if (cmsg.data.c[5] >= 0x20): print "%c" % cmsg.data.c[5]
					else: print " "
				elif cmsg.data.c[0] == SDO_SEG_RES_INIT_UPLOAD_3BYTE:      
					data = cmsg.data.l[1] & 0xFFFFFF
					print "data: %d  0x%0.6X" % (data,data),
					if (cmsg.data.c[4] >= 0x20): print "%c" % cmsg.data.c[4],
					else: print " ",
					if (cmsg.data.c[5] >= 0x20): print "%c" % cmsg.data.c[5],
					else: print " ",
					if (cmsg.data.c[6] >= 0x20): print "%c" % cmsg.data.c[6]
					else: print " "
				elif cmsg.data.c[0] == SDO_SEG_RES_INIT_UPLOAD_4BYTE:      
					data = cmsg.data.l[1] 
					print "data: %d  0x%0.8X" % (data,data),
					if (cmsg.data.c[4] >= 0x20): print "%c" % cmsg.data.c[4],
					else: print " ",
					if (cmsg.data.c[5] >= 0x20): print "%c" % cmsg.data.c[5],
					else: print " ",
					if (cmsg.data.c[6] >= 0x20): print "%c" % cmsg.data.c[6],
					else: print " ",
					if (cmsg.data.c[7] >= 0x20): print "%c" % cmsg.data.c[7]
					else: print " "
				else: 
					print "Unknown Command: 0x%0.2X" %  cmsg.data.c[0]
					result = False
#			print cif.msg_count 
		else:
			print "unknown"
			result = False
	except IOError, (errno):
			print "I/O error(%s): " % (errno)
			result = False
		
	cif.canIdDelete(SDOTX_ID+node)
	del cmsg
	return result,data


def SDOwrite1Byte (cif,node,index,subindex,data,result=True):
	cmsg = ntcan.CMSG()
	rdindex = 0
	cif.canIdAdd(SDOTX_ID+node)
	print "SDOwrite1Byte node: %d index: 0x%0.4X subindex: 0x%0.2X data: 0x%0.2X -> " %(node,index,subindex,data),
	cmsg.canWriteByte(cif, SDORX_ID+node,8, SDO_SEG_REQ_INIT_DOWNLOAD_1BYTE,index & 0xFF,(index & 0xFF00)>>8,subindex,data,0,0,0)
	data=0
	try: 
		cmsg.canRead(cif)
#		print cmsg 
		if cmsg.len == 8:
			rdindex = cmsg.data.c[1] + cmsg.data.c[2] *256
			if (rdindex == index and cmsg.data.c[3] == subindex):
				if cmsg.data.c[0] == SDO_SEG_ABORT_TRANSFER:
					print "error: 0x%0.8X %s" % (cmsg.data.l[1],SDOerror.get(cmsg.data.l[1]))
					data=cmsg.data.l[1]
					result=False
				elif cmsg.data.c[0] == SDO_SEG_RES_INIT_DOWNLOAD:      
					print "Ok"
				else: 
					print "Unknown Command: 0x%0.2X" %  cmsg.data.c[0]
					result=False
		else:
			print "unknown"
			result=False

	except IOError, (errno):
		print "I/O error(%s): " % (errno)
		result=False

	cif.canIdDelete(SDOTX_ID+node)
	del cmsg
	return result,data

def SDOwrite2Byte (cif,node,index,subindex,data,result=True):
	cmsg = ntcan.CMSG()
	rdindex = 0
	cif.canIdAdd(SDOTX_ID+node)
	print "SDOwrite2Byte node: %d index: 0x%0.4X subindex: 0x%0.2X data: 0x%0.4X -> " %(node,index,subindex,data),
	cmsg.canWriteByte(cif, SDORX_ID+node,8, SDO_SEG_REQ_INIT_DOWNLOAD_2BYTE,index & 0xFF,(index & 0xFF00)>>8,subindex,data&0xFF,(data&0xFF00)>>8,0,0)
	data=0
	try: 
		cmsg.canRead(cif)
#		print cmsg 
		if cmsg.len == 8:
			rdindex = cmsg.data.c[1] + cmsg.data.c[2] *256
			if (rdindex == index and cmsg.data.c[3] == subindex):
				if cmsg.data.c[0] == SDO_SEG_ABORT_TRANSFER:
					print "error: 0x%0.8X %s" % (cmsg.data.l[1],SDOerror.get(cmsg.data.l[1]))
					data=cmsg.data.l[1]
					result=False
				elif cmsg.data.c[0] == SDO_SEG_RES_INIT_DOWNLOAD:      
					print "Ok"
				else: 
					print "Unknown Command: 0x%0.2X" %  cmsg.data.c[0]
					result=False
		else:
			print "unknown"
			result=False

	except IOError, (errno):
		print "I/O error(%s): " % (errno)
		result=False
		    
	cif.canIdDelete(SDOTX_ID+node)
	del cmsg
	return result,data

def SDOwrite4Byte (cif,node,index,subindex,data,result=True):
	cmsg = ntcan.CMSG()
	rdindex = 0
	cif.canIdAdd(SDOTX_ID+node)
	print "SDOwrite4Byte node: %d index: 0x%0.4X subindex: 0x%0.2X data: 0x%0.8X -> " %(node,index,subindex,data),
	#cmsg.canWriteByte(cif, SDORX_ID+node,8, SDO_SEG_REQ_INIT_DOWNLOAD_4BYTE,index & 0xFF,(index & 0xFF00)>>8,subindex,data&0x000000FF,(data&0x0000FF00)>>8,(data&0x00FF0000)>>16,(data&0xFF000000)>>24)
	cmsg.canWriteByte(cif, SDORX_ID+node,8, SDO_SEG_REQ_INIT_DOWNLOAD_4BYTE,index & 0xFF,(index & 0xFF00)>>8,subindex,data&0xFF,(data>>8)&0xFF,(data>>16)&0xFF,(data>>24)&0xFF)
	data=0
	try: 
		cmsg.canRead(cif)
#		print cmsg 
		if cmsg.len == 8:
			rdindex = cmsg.data.c[1] + cmsg.data.c[2] *256
#			print "index: %x  %x  %x" % (rdindex,cmsg.data.c[1],cmsg.data.c[2]*256)
			if (rdindex == index and cmsg.data.c[3] == subindex):
				if cmsg.data.c[0] == SDO_SEG_ABORT_TRANSFER:
					print "error: 0x%0.8X %s" % (cmsg.data.l[1],SDOerror.get(cmsg.data.l[1]))
					data=cmsg.data.l[1]
					result=False
				elif cmsg.data.c[0] == SDO_SEG_RES_INIT_DOWNLOAD:      
					print "Ok"
				else: 
					print "Unknown Command: 0x%0.2X" %  cmsg.data.c[0]
					result=False
		else:
			print "unknown"
			result=False

	except IOError, (errno):
		print "I/O error(%s): " % (errno)
		result=False
	cif.canIdDelete(SDOTX_ID+node)
	del cmsg
	return result,data

def SDOwriteString (cif,node,index,subindex,str,data=0,result=True):
	cmsg = ntcan.CMSG()
	rdindex = 0
	cif.canIdAdd(SDOTX_ID+node)
	print "SDOwriteString node: %d index: 0x%0.4X subindex: 0x%0.2X data: %s -> " %(node,index,subindex,str),
	
	if len(str) == 0 or len(str) > 4:
		print "Wrong string!"
		result=False
	else: 
		if len(str) == 1:
			cmsg.canWriteByte(cif, SDORX_ID+node,8, SDO_SEG_REQ_INIT_DOWNLOAD_1BYTE,index & 0xFF,(index & 0xFF00)>>8,subindex,ord(str[0]),0,0,0)	
		elif len(str) == 2:
			cmsg.canWriteByte(cif, SDORX_ID+node,8, SDO_SEG_REQ_INIT_DOWNLOAD_2BYTE,index & 0xFF,(index & 0xFF00)>>8,subindex,ord(str[0]),ord(str[1]),0,0)	
		elif len(str) == 3:
			cmsg.canWriteByte(cif, SDORX_ID+node,8, SDO_SEG_REQ_INIT_DOWNLOAD_3BYTE,index & 0xFF,(index & 0xFF00)>>8,subindex,ord(str[0]),ord(str[1]),ord(str[2]),0)	
		elif len(str) == 4:
			cmsg.canWriteByte(cif, SDORX_ID+node,8, SDO_SEG_REQ_INIT_DOWNLOAD_4BYTE,index & 0xFF,(index & 0xFF00)>>8,subindex,ord(str[0]),ord(str[1]),ord(str[2]),ord(str[3]))	
		data=0
		try: 
			cmsg.canRead(cif)
			if cmsg.len == 8:
				rdindex = cmsg.data.c[1] + cmsg.data.c[2] *256
				if (rdindex == index and cmsg.data.c[3] == subindex):
					if cmsg.data.c[0] == SDO_SEG_ABORT_TRANSFER:
						print "error: 0x%0.8X %s" % (cmsg.data.l[1],SDOerror.get(cmsg.data.l[1]))
						data=cmsg.data.l[1]
						result=False
					elif cmsg.data.c[0] == SDO_SEG_RES_INIT_DOWNLOAD:      
						print "Ok"
					else: 
						print "Unknown Command: 0x%0.2X" %  cmsg.data.c[0]
						result=False
			else:
				print "unknown"
	
		except IOError, (errno):
			print "I/O error(%s): " % (errno)
			result=False
	cif.canIdDelete(SDOTX_ID+node)
	del cmsg
	return result,data



class SDO:
	def __init__(self,net,baudrate,RxQS,RxTO,TxQS,TxTO):
		self.cif = ntcan.CIF(net,RxQS,RxTO,TxQS,TxTO)
		self.cif.baudrate = baudrate
		self.cmsg = ntcan.CMSG()
		#del cmsg
	def read (self,node,index,subindex,data=0,result=True):
		rdindex = 0
		lstr = [] #empty 
		self.cif.canIdAdd(SDOTX_ID+node)
		print "SDOread node: %d index: 0x%0.4X subindex: 0x%0.2X -> " %(node,index,subindex),
		self.cmsg.canWriteByte(self.cif, SDORX_ID+node,8, SDO_SEG_REQ_INIT_UPLOAD,index & 0xFF,(index & 0xFF00)>>8,subindex,0,0,0,0)
		try: 
			self.cmsg.canRead(self.cif)
			if self.cmsg.len == 8:
				rdindex = self.cmsg.data.c[1] + self.cmsg.data.c[2] *256
				if (rdindex == index and self.cmsg.data.c[3] == subindex):
					if self.cmsg.data.c[0] == SDO_SEG_ABORT_TRANSFER:
						print "error: 0x%0.8X %s" % (self.cmsg.data.l[1],SDOerror.get(self.cmsg.data.l[1]))
						result = False
						data = self.cmsg.data.l[1]
					elif self.cmsg.data.c[0] == SDO_SEG_RES_INIT_UPLOAD_xBYTE:      
						data = self.cmsg.data.l[1]
						print "data: %d  0x%0.8X" % (data,data)
					elif self.cmsg.data.c[0] == SDO_SEG_RES_INIT_UPLOAD_1BYTE:      
						data = self.cmsg.data.l[1] & 0xFF
						print "data: %d  0x%0.2X" % (data,data),
						if (self.cmsg.data.c[4] >= 0x20): print "%c" % self.cmsg.data.c[4]
						else: print " "
					elif self.cmsg.data.c[0] == SDO_SEG_RES_INIT_UPLOAD_2BYTE:      
						data = self.cmsg.data.l[1] & 0xFFFF
						print "data: %d  0x%0.4X" % (data,data),
						if (self.cmsg.data.c[4] >= 0x20): print "%c" % self.cmsg.data.c[4],
						else: print " ",
						if (self.cmsg.data.c[5] >= 0x20): print "%c" % self.cmsg.data.c[5]
						else: print " "
					elif self.cmsg.data.c[0] == SDO_SEG_RES_INIT_UPLOAD_3BYTE:      
						data = self.cmsg.data.l[1] & 0xFFFFFF
						print "data: %d  0x%0.6X" % (data,data),
						if (self.cmsg.data.c[4] >= 0x20): print "%c" % self.cmsg.data.c[4],
						else: print " ",
						if (self.cmsg.data.c[5] >= 0x20): print "%c" % self.cmsg.data.c[5],
						else: print " ",
						if (self.cmsg.data.c[6] >= 0x20): print "%c" % self.cmsg.data.c[6]
						else: print " "
					elif self.cmsg.data.c[0] == SDO_SEG_RES_INIT_UPLOAD_4BYTE:      
						data = self.cmsg.data.l[1] 
						print "data: %d  0x%0.8X" % (data,data),
						if (self.cmsg.data.c[4] >= 0x20): print "%c" % self.cmsg.data.c[4],
						else: print " ",
						if (self.cmsg.data.c[5] >= 0x20): print "%c" % self.cmsg.data.c[5],
						else: print " ",
						if (self.cmsg.data.c[6] >= 0x20): print "%c" % self.cmsg.data.c[6],
						else: print " ",
						if (self.cmsg.data.c[7] >= 0x20): print "%c" % self.cmsg.data.c[7]
						else: print " "
					elif SDO_SEG_RES_INIT_UPLOAD_nBYTE==0x41:
						len = self.cmsg.data.l[1]
						cmd=0x60
						while len>0:
							#print len
							self.cmsg.canWriteByte(self.cif, SDORX_ID+node,8, cmd)
							try: 
								self.cmsg.canRead(self.cif)
								if ((self.cmsg.data.c[0]&0x10) != (cmd&0x10)):
									self.__error(node,index,subindex,0x05030000) # toggle bit wrong
									break
								else:	
									if (self.cmsg.data.c[0] & 0x01): 		# last data
										"""
										if (self.cmsg.data.c[0] & 0x0E) == 0:
											pass # no data
										else:
										"""
										len = 7 - ((self.cmsg.data.c[0] & 0x0E)>>1)
										for i in range(len):
											lstr.append(chr(self.cmsg.data.c[i+1]))
										break	
									else:
										len = len-7
										for i in range(7):
											lstr.append(chr(self.cmsg.data.c[i+1]))
							except IOError, (errno):
									print "I/O error(%s): " % (errno)
									result = False
							if cmd==0x60: cmd=0x70
							else: cmd=0x60
						data = "".join(lstr)
						print data
					elif SDO_SEG_RES_INIT_UPLOAD_xBYTE==0x42:
						print "Download not supported! Command: 0x%0.2X" %  self.cmsg.data.c[0]
						result = False
					else: 
						print "Unknown Command: 0x%0.2X" %  self.cmsg.data.c[0]
						result = False
	#			print self.cif.msg_count 
			else:
				print "unknown"
				result = False
		except IOError, (errno):
				print "I/O error(%s): " % (errno)
				result = False
		self.cif.canIdDelete(SDOTX_ID+node)
		return result,data
	def write1Byte (self,node,index,subindex,data,result=True):
		rdindex = 0
		self.cif.canIdAdd(SDOTX_ID+node)
		print "SDOwrite1Byte node: %d index: 0x%0.4X subindex: 0x%0.2X data: 0x%0.2X -> " %(node,index,subindex,data),
		self.cmsg.canWriteByte(self.cif, SDORX_ID+node,8, SDO_SEG_REQ_INIT_DOWNLOAD_1BYTE,index & 0xFF,(index & 0xFF00)>>8,subindex,data,0,0,0)
		data=0
		try: 
			self.cmsg.canRead(self.cif)
	#		print self.cmsg 
			if self.cmsg.len == 8:
				rdindex = self.cmsg.data.c[1] + self.cmsg.data.c[2] *256
				if (rdindex == index and self.cmsg.data.c[3] == subindex):
					if self.cmsg.data.c[0] == SDO_SEG_ABORT_TRANSFER:
						print "error: 0x%0.8X %s" % (self.cmsg.data.l[1],SDOerror.get(self.cmsg.data.l[1]))
						data=self.cmsg.data.l[1]
						result=False
					elif self.cmsg.data.c[0] == SDO_SEG_RES_INIT_DOWNLOAD:      
						print "Ok"
					else: 
						print "Unknown Command: 0x%0.2X" %  self.cmsg.data.c[0]
						result=False
			else:
				print "unknown"
				result=False
	
		except IOError, (errno):
			print "I/O error(%s): " % (errno)
			result=False
	
		self.cif.canIdDelete(SDOTX_ID+node)
		return result,data
	def write2Byte (self,node,index,subindex,data,result=True):
		rdindex = 0
		self.cif.canIdAdd(SDOTX_ID+node)
		print "SDOwrite2Byte node: %d index: 0x%0.4X subindex: 0x%0.2X data: 0x%0.4X -> " %(node,index,subindex,data),
		self.cmsg.canWriteByte(self.cif, SDORX_ID+node,8, SDO_SEG_REQ_INIT_DOWNLOAD_2BYTE,index & 0xFF,(index & 0xFF00)>>8,subindex,data&0xFF,(data&0xFF00)>>8,0,0)
		data=0
		try: 
			self.cmsg.canRead(self.cif)
	#		print self.cmsg 
			if self.cmsg.len == 8:
				rdindex = self.cmsg.data.c[1] + self.cmsg.data.c[2] *256
				if (rdindex == index and self.cmsg.data.c[3] == subindex):
					if self.cmsg.data.c[0] == SDO_SEG_ABORT_TRANSFER:
						print "error: 0x%0.8X %s" % (self.cmsg.data.l[1],SDOerror.get(self.cmsg.data.l[1]))
						data=self.cmsg.data.l[1]
						result=False
					elif self.cmsg.data.c[0] == SDO_SEG_RES_INIT_DOWNLOAD:      
						print "Ok"
					else: 
						print "Unknown Command: 0x%0.2X" %  self.cmsg.data.c[0]
						result=False
			else:
				print "unknown"
				result=False
	
		except IOError, (errno):
			print "I/O error(%s): " % (errno)
			result=False
			    
		self.cif.canIdDelete(SDOTX_ID+node)
		return result,data
	def write4Byte (self,node,index,subindex,data,result=True):
		rdindex = 0
		self.cif.canIdAdd(SDOTX_ID+node)
		print "SDOwrite4Byte node: %d index: 0x%0.4X subindex: 0x%0.2X data: 0x%0.8X -> " %(node,index,subindex,data),
		#self.cmsg.canWriteByte(self.cif, SDORX_ID+node,8, SDO_SEG_REQ_INIT_DOWNLOAD_4BYTE,index & 0xFF,(index & 0xFF00)>>8,subindex,data&0x000000FF,(data&0x0000FF00)>>8,(data&0x00FF0000)>>16,(data&0xFF000000)>>24)
		self.cmsg.canWriteByte(self.cif, SDORX_ID+node,8, SDO_SEG_REQ_INIT_DOWNLOAD_4BYTE,index & 0xFF,(index & 0xFF00)>>8,subindex,data&0xFF,(data>>8)&0xFF,(data>>16)&0xFF,(data>>24)&0xFF)
		
		data=0
		try: 
			self.cmsg.canRead(self.cif)
	#		print self.cmsg 
			if self.cmsg.len == 8:
				rdindex = self.cmsg.data.c[1] + self.cmsg.data.c[2] *256
				if (rdindex == index and self.cmsg.data.c[3] == subindex):
					if self.cmsg.data.c[0] == SDO_SEG_ABORT_TRANSFER:
						print "error: 0x%0.8X %s" % (self.cmsg.data.l[1],SDOerror.get(self.cmsg.data.l[1]))
						data=self.cmsg.data.l[1]
						result=False
					elif self.cmsg.data.c[0] == SDO_SEG_RES_INIT_DOWNLOAD:      
						print "Ok"
					else: 
						print "Unknown Command: 0x%0.2X" %  self.cmsg.data.c[0]
						result=False
			else:
				print "unknown"
				result=False
	
		except IOError, (errno):
			print "I/O error(%s): " % (errno)
			result=False
		self.cif.canIdDelete(SDOTX_ID+node)
		return result,data
	def writeString (self,node,index,subindex,str,data=0,result=True):
		rdindex = 0
		self.cif.canIdAdd(SDOTX_ID+node)
		print "SDOwriteString node: %d index: 0x%0.4X subindex: 0x%0.2X data: %s -> " %(node,index,subindex,str),
		if len(str) == 0 or len(str) > 4:
			print "Wrong string!"
			result=False
		else: 
			if len(str) == 1:
				self.cmsg.canWriteByte(self.cif, SDORX_ID+node,8, SDO_SEG_REQ_INIT_DOWNLOAD_1BYTE,index & 0xFF,(index & 0xFF00)>>8,subindex,ord(str[0]),0,0,0)	
			elif len(str) == 2:
				self.cmsg.canWriteByte(self.cif, SDORX_ID+node,8, SDO_SEG_REQ_INIT_DOWNLOAD_2BYTE,index & 0xFF,(index & 0xFF00)>>8,subindex,ord(str[0]),ord(str[1]),0,0)	
			elif len(str) == 3:
				self.cmsg.canWriteByte(self.cif, SDORX_ID+node,8, SDO_SEG_REQ_INIT_DOWNLOAD_3BYTE,index & 0xFF,(index & 0xFF00)>>8,subindex,ord(str[0]),ord(str[1]),ord(str[2]),0)	
			elif len(str) == 4:
				self.cmsg.canWriteByte(self.cif, SDORX_ID+node,8, SDO_SEG_REQ_INIT_DOWNLOAD_4BYTE,index & 0xFF,(index & 0xFF00)>>8,subindex,ord(str[0]),ord(str[1]),ord(str[2]),ord(str[3]))	
			data=0
			try: 
				self.cmsg.canRead(self.cif)
				if self.cmsg.len == 8:
					rdindex = self.cmsg.data.c[1] + self.cmsg.data.c[2] *256
					if (rdindex == index and self.cmsg.data.c[3] == subindex):
						if self.cmsg.data.c[0] == SDO_SEG_ABORT_TRANSFER:
							print "error: 0x%0.8X %s" % (self.cmsg.data.l[1],SDOerror.get(self.cmsg.data.l[1]))
							data=self.cmsg.data.l[1]
							result=False
						elif self.cmsg.data.c[0] == SDO_SEG_RES_INIT_DOWNLOAD:      
							print "Ok"
						else: 
							print "Unknown Command: 0x%0.2X" %  self.cmsg.data.c[0]
							result=False
				else:
					print "unknown"
		
			except IOError, (errno):
				print "I/O error(%s): " % (errno)
				result=False
		self.cif.canIdDelete(SDOTX_ID+node)
		return result,data
	def __error (self,node,index,subindex,error,result=True):
		rdindex = 0
		print "error: 0x%0.8X %s" %(error,SDOerror.get(error))
		#self.cmsg.canWriteByte(self.cif, SDORX_ID+node,8, SDO_SEG_REQ_INIT_DOWNLOAD_4BYTE,index & 0xFF,(index & 0xFF00)>>8,subindex,error&0xFF,(error>>8)&0xFF,(error>>16)&0xFF,(error>>24)&0xFF)
		return result
	def scan (self,node,startindex=0,stopindex=0,data=0,result=True):
		self.cif.canIdAdd(SDOTX_ID+node)
		#print "SDOscan node: %d index: 0x%0.4X subindex: 0x%0.2X -> " %(node,index,subindex),
		for index in range (startindex,stopindex+1,1):
			rdindex = 0
			subindex=0
			result = True
			while result == True:
				lstr = [] #empty 
				self.cmsg.canWriteByte(self.cif, SDORX_ID+node,8, SDO_SEG_REQ_INIT_UPLOAD,index & 0xFF,(index & 0xFF00)>>8,subindex,0,0,0,0)
				try: 
					self.cmsg.canRead(self.cif)
					if self.cmsg.len == 8:
						rdindex = self.cmsg.data.c[1] + self.cmsg.data.c[2] *256
						if (rdindex == index and self.cmsg.data.c[3] == subindex):
							if self.cmsg.data.c[0] == SDO_SEG_ABORT_TRANSFER:
		#							print "error: 0x%0.8X %s" % (self.cmsg.data.l[1],SDOerror.get(self.cmsg.data.l[1]))
								result = False
								data = self.cmsg.data.l[1]
							elif self.cmsg.data.c[0] == SDO_SEG_RES_INIT_UPLOAD_xBYTE:      
								data = self.cmsg.data.l[1]
								print "SDOscan node: %d index: 0x%0.4X subindex: 0x%0.2X -> " %(node,index,subindex),
								print "data: %d  0x%0.8X" % (data,data)
							elif self.cmsg.data.c[0] == SDO_SEG_RES_INIT_UPLOAD_1BYTE:      
								data = self.cmsg.data.l[1] & 0xFF
								print "SDOscan node: %d index: 0x%0.4X subindex: 0x%0.2X -> " %(node,index,subindex),
								print "data: %d  0x%0.2X" % (data,data),
								if (self.cmsg.data.c[4] >= 0x20): print "%c" % self.cmsg.data.c[4]
								else: print " "
							elif self.cmsg.data.c[0] == SDO_SEG_RES_INIT_UPLOAD_2BYTE:      
								data = self.cmsg.data.l[1] & 0xFFFF
								print "SDOscan node: %d index: 0x%0.4X subindex: 0x%0.2X -> " %(node,index,subindex),
								print "data: %d  0x%0.4X" % (data,data),
								if (self.cmsg.data.c[4] >= 0x20): print "%c" % self.cmsg.data.c[4],
								else: print " ",
								if (self.cmsg.data.c[5] >= 0x20): print "%c" % self.cmsg.data.c[5]
								else: print " "
							elif self.cmsg.data.c[0] == SDO_SEG_RES_INIT_UPLOAD_3BYTE:      
								data = self.cmsg.data.l[1] & 0xFFFFFF
								print "SDOscan node: %d index: 0x%0.4X subindex: 0x%0.2X -> " %(node,index,subindex),
								print "data: %d  0x%0.6X" % (data,data),
								if (self.cmsg.data.c[4] >= 0x20): print "%c" % self.cmsg.data.c[4],
								else: print " ",
								if (self.cmsg.data.c[5] >= 0x20): print "%c" % self.cmsg.data.c[5],
								else: print " ",
								if (self.cmsg.data.c[6] >= 0x20): print "%c" % self.cmsg.data.c[6]
								else: print " "
							elif self.cmsg.data.c[0] == SDO_SEG_RES_INIT_UPLOAD_4BYTE:      
								data = self.cmsg.data.l[1] 
								print "SDOscan node: %d index: 0x%0.4X subindex: 0x%0.2X -> " %(node,index,subindex),
								print "data: %d  0x%0.8X" % (data,data),
								if (self.cmsg.data.c[4] >= 0x20): print "%c" % self.cmsg.data.c[4],
								else: print " ",
								if (self.cmsg.data.c[5] >= 0x20): print "%c" % self.cmsg.data.c[5],
								else: print " ",
								if (self.cmsg.data.c[6] >= 0x20): print "%c" % self.cmsg.data.c[6],
								else: print " ",
								if (self.cmsg.data.c[7] >= 0x20): print "%c" % self.cmsg.data.c[7]
								else: print " "
							elif SDO_SEG_RES_INIT_UPLOAD_nBYTE==0x41:
								len = self.cmsg.data.l[1]
								cmd=0x60
								print "SDOscan node: %d index: 0x%0.4X subindex: 0x%0.2X -> " %(node,index,subindex),
								while len>0:
									self.cmsg.canWriteByte(self.cif, SDORX_ID+node,8, cmd)
									try: 
										self.cmsg.canRead(self.cif)
										if ((self.cmsg.data.c[0]&0x10) != (cmd&0x10)):
											self.__error(node,index,subindex,0x05030000) # toggle bit wrong
											break
										else:	
											if (self.cmsg.data.c[0] & 0x01): 		# last data
												len = 7 - ((self.cmsg.data.c[0] & 0x0E)>>1)
												for i in range(len):
													lstr.append(chr(self.cmsg.data.c[i+1]))
												break	
											else:
												len = len-7
												for i in range(7):
													lstr.append(chr(self.cmsg.data.c[i+1]))
									except IOError, (errno):
											print "I/O error(%s): " % (errno)
											result = False
									if cmd==0x60: cmd=0x70
									else: cmd=0x60
								data = "".join(lstr)
								print data
							elif SDO_SEG_RES_INIT_UPLOAD_xBYTE==0x42:
								print "Download not supported! Command: 0x%0.2X" %  self.cmsg.data.c[0]
								result = False
							else: 
								print "Unknown Command: 0x%0.2X" %  self.cmsg.data.c[0]
								result = False
			#			print self.cif.msg_count 
					else:
						print "unknown"
						result = False
				except IOError, (errno):
					print "I/O error(%s): " % (errno)
					result = False
					return result,data
				subindex = subindex + 1	
					
		self.cif.canIdDelete(SDOTX_ID+node)
		return result,data
	


# PDO -------------------------------------------------------------------------
def PDO1writeByte (cif,node,len,d0=0,d1=0,d2=0,d3=0,d4=0,d5=0,d6=0,d7=0):
	cmsg = ntcan.CMSG()
	print "PDO1writeByte node: %d -> " %(node),
	cmsg.canWriteByte(cif, PDO1RX_ID+node,len,d0,d1,d2,d3,d4,d5,d6,d7)
	print cmsg
	del cmsg

def PDO1writeShort (cif,node,len,d0=0,d1=0,d2=0,d3=0):
	cmsg = ntcan.CMSG()
	print "PDO1writeShort node: %d -> " %(node),
	cmsg.canWriteShort(cif, PDO1RX_ID+node,len,d0,d1,d2,d3)
	print cmsg
	del cmsg

def PDO1writeLong (cif,node,len,d0=0,d1=0):
	cmsg = ntcan.CMSG()
	print "PDO1writeLong node: %d -> " %(node),
	cmsg.canWriteLong(cif, PDO1RX_ID+node,len,d0,d1)
	print cmsg
	del cmsg

def PDO2writeByte (cif,node,len,d0=0,d1=0,d2=0,d3=0,d4=0,d5=0,d6=0,d7=0):
	cmsg = ntcan.CMSG()
	print "PDO2writeByte node: %d -> " %(node),
	cmsg.canWriteByte(cif, PDO2RX_ID+node,len,d0,d1,d2,d3,d4,d5,d6,d7)
	print cmsg
	del cmsg

def PDO2writeShort (cif,node,len,d0=0,d1=0,d2=0,d3=0):
	cmsg = ntcan.CMSG()
	print "PDO2writeShort node: %d -> " %(node),
	cmsg.canWriteShort(cif, PDO1RX_ID+node,len,d0,d1,d2,d3)
	print cmsg
	del cmsg

def PDO2writeLong (cif,node,len,d0=0,d1=0):
	cmsg = ntcan.CMSG()
	print "PDO2writeLong node: %d -> " %(node),
	cmsg.canWriteLong(cif, PDO2RX_ID+node,len,d0,d1)
	print cmsg
	del cmsg

def PDO3writeByte (cif,node,len,d0=0,d1=0,d2=0,d3=0,d4=0,d5=0,d6=0,d7=0):
	cmsg = ntcan.CMSG()
	print "PDO3writeByte node: %d -> " %(node),
	cmsg.canWriteByte(cif, PDO3RX_ID+node,len,d0,d1,d2,d3,d4,d5,d6,d7)
	print cmsg
	del cmsg

def PDO3writeShort (cif,node,len,d0=0,d1=0,d2=0,d3=0):
	cmsg = ntcan.CMSG()
	print "PDO3writeShort node: %d -> " %(node),
	cmsg.canWriteShort(cif, PDO3RX_ID+node,len,d0,d1,d2,d3)
	print cmsg
	del cmsg

def PDO3writeLong (cif,node,len,d0=0,d1=0):
	cmsg = ntcan.CMSG()
	print "PDO3writeLong node: %d -> " %(node),
	cmsg.canWriteLong(cif, PDO3RX_ID+node,len,d0,d1)
	print cmsg
	del cmsg

def PDO4writeByte (cif,node,len,d0=0,d1=0,d2=0,d3=0,d4=0,d5=0,d6=0,d7=0):
	cmsg = ntcan.CMSG()
	print "PDO4writeByte node: %d -> " %(node),
	cmsg.canWriteByte(cif, PDO4RX_ID+node,len,d0,d1,d2,d3,d4,d5,d6,d7)
	print cmsg
	del cmsg

def PDO4writeShort (cif,node,len,d0=0,d1=0,d2=0,d3=0):
	cmsg = ntcan.CMSG()
	print "PDO4writeShort node: %d -> " %(node),
	cmsg.canWriteShort(cif, PDO4RX_ID+node,len,d0,d1,d2,d3)
	print cmsg
	del cmsg

def PDO4writeLong (cif,node,len,d0=0,d1=0):
	cmsg = ntcan.CMSG()
	print "PDO4writeLong node: %d -> " %(node),
	cmsg.canWriteLong(cif, PDO4RX_ID+node,len,d0,d1)
	print cmsg
	del cmsg


def PDO1read (cif,node,cmsg=0,result=True):
	lcmsg=ntcan.CMSG()
	cif.canIdAdd(PDO1TX_ID+node)
	print "PDO1read node: %d -> " %(node),
	lcmsg.canWriteByte(cif, PDO1TX_ID+node,0x10)
	try: 
		lcmsg.canRead(cif)
		print lcmsg 
	except IOError, (errno):
		    print "I/O error(%s): " % (errno)
		    result = False
	cif.canIdDelete(PDO1TX_ID+node)
	cmsg = lcmsg
	del lcmsg
	return result,cmsg

def PDO2read (cif,node,cmsg=0,result=True):
	lcmsg=ntcan.CMSG()
	cif.canIdAdd(PDO2TX_ID+node)
	print "PDO2read node: %d -> " %(node),
	lcmsg.canWriteByte(cif, PDO2TX_ID+node,0x10)
	try: 
		lcmsg.canRead(cif)
		print lcmsg 
	except IOError, (errno):
		    print "I/O error(%s): " % (errno)
		    result = False
	cif.canIdDelete(PDO2TX_ID+node)
	cmsg = lcmsg
	del lcmsg
	return result,cmsg

def PDO3read (cif,node,cmsg=0,result=True):
	lcmsg=ntcan.CMSG()
	cif.canIdAdd(PDO3TX_ID+node)
	print "PDO3read node: %d -> " %(node),
	lcmsg.canWriteByte(cif, PDO3TX_ID+node,0x10)
	try: 
		lcmsg.canRead(cif)
		print lcmsg 
	except IOError, (errno):
		    print "I/O error(%s): " % (errno)
		    result = False
	cif.canIdDelete(PDO3TX_ID+node)
	cmsg = lcmsg
	del lcmsg
	return result,cmsg

def PDO4read (cif,node,cmsg=0,result=True):
	lcmsg=ntcan.CMSG()
	cif.canIdAdd(PDO4TX_ID+node)
	print "PDO4read node: %d -> " %(node),
	lcmsg.canWriteByte(cif, PDO4TX_ID+node,0x10)
	try: 
		lcmsg.canRead(cif)
		print lcmsg 
	except IOError, (errno):
		    print "I/O error(%s): " % (errno)
		    result = False
	cif.canIdDelete(PDO4TX_ID+node)
	cmsg = lcmsg
	del lcmsg
	return result,cmsg

	

class PDO:
	def __init__(self,obj,net,baudrate,RxQS,RxTO,TxQS,TxTO):
		if obj>4 or obj <1:
			print "PDO -> wrong obj %d -> " %(obj)
			return
		self.cif = ntcan.CIF(net,RxQS,RxTO,TxQS,TxTO)
		self.cif.baudrate = baudrate
		self.cmsg = ntcan.CMSG()
		self.obj = obj
		self.txid = PDO1TX_ID + (obj-1)*0x100
		self.rxid = PDO1RX_ID + (obj-1)*0x100
		#print "obj %d txid: 0x%0.3X rxid: 0x%0.3X"%(self.obj,self.txid,self.rxid)
		#del cmsg

	def __splitdata(self,data,len):
		pos = 0
		rdata=[0,0,0,0,0,0,0,0]
		if len==1:
			rdata[pos]=data&0xFF
		elif len==2:
			rdata[pos+1]=(data>>8)&0xFF
			rdata[pos+0]=(data>>0)&0xFF
		elif len==3:
			rdata[pos+2]=(data>>16)&0xFF
			rdata[pos+1]=(data>>8)&0xFF
			rdata[pos+0]=(data>>0)&0xFF
		elif len==4:
			rdata[pos+3]=(data>>24)&0xFF
			rdata[pos+2]=(data>>16)&0xFF
			rdata[pos+1]=(data>>8)&0xFF
			rdata[pos+0]=(data>>0)&0xFF
		return rdata
		
	def write (self,node,d0=0,l0=0,d1=0,l1=0,d2=0,l2=0,d3=0,l3=0,d4=0,l4=0,d5=0,l5=0,d6=0,l6=0,d7=0,l7=0):
		print "PDO%dwrite node: %d -> " %(self.obj,node),
		len = l0+l1+l2+l3+l4+l6+l7
		if (len > 8 or len < 0):
			print "Error: wrong data length (%d)" % (len)
		else:
			pos=0
			_datalen=[l0,l1,l2,l3,l4,l5,l6,l7]
			_data=[d0,d1,d2,d3,d4,d5,d6,d7]
#			print _datalen
#			print _data
			_candata=[0,0,0,0,0,0,0,0]
			for x in range(8):
				_candata[pos:pos+_datalen[x]]=self.__splitdata(_data[x],_datalen[x])
				pos = pos + _datalen[x]
#				print x,_datalen[x],_data[x],pos
#				print _candata
			self.cmsg.canWriteByte(self.cif,(self.rxid+node),len,_candata[0],_candata[1],_candata[2],_candata[3],_candata[4],_candata[5],_candata[6],_candata[7])
			print self.cmsg
	def writeByte (self,node,len,d0=0,d1=0,d2=0,d3=0,d4=0,d5=0,d6=0,d7=0):
		print "PDO%dwriteByte node: %d -> " %(self.obj,node),
		self.cmsg.canWriteByte(self.cif,(self.rxid+node),len,d0,d1,d2,d3,d4,d5,d6,d7)
		print self.cmsg
	def writeShort (self,node,len,d0=0,d1=0,d2=0,d3=0):
		print "PDO%dwriteShort node: %d -> " %(self.obj,node),
		self.cmsg.canWriteShort(self.cif,(self.rxid+node),len,d0,d1,d2,d3)
		print self.cmsg
	def writeLong (self,node,len,d0=0,d1=0):
		print "PDO%dwriteLong node: %d -> " %(self.obj,node),
		self.cmsg.canWriteLong(self.cif,(self.rxid+node),len,d0,d1)
		print self.cmsg
	def writeString (self,node,str):
		print "PDO%dwriteString node: %d -> " %(self.obj,node),
		strlen = len(str)
		i = 0
		#a = (((strlen/8)*8)+8)
		#print a
		str = string.ljust(str,(((strlen/8)*8)+8))
		#print "#%s#"%(str)
		while (strlen>0):
			if strlen > 7: pdolen = 8
			else: pdolen=strlen%8
			self.cmsg.canWriteByte(self.cif, (self.rxid+node),pdolen, ord(str[i]),ord(str[i+1]),ord(str[i+2]),ord(str[i+3]),ord(str[i+4]),ord(str[i+5]),ord(str[i+6]),ord(str[i+7]))	
			strlen = strlen-8
			i=i+8
			#print "%d %d %d"%(pdolen,strlen,i),
		print str #self.cmsg
	def read (self,node,cmsg=0,result=True,silent=False):
		self.cif.canIdAdd(self.txid+node)
		if (silent): print "PDO%dread node: %d -> " %(self.obj,node),
		self.cmsg.canWriteByte(self.cif,(self.txid+node),0x10)    #rtr
		try: 
			self.cmsg.canRead(self.cif)
			if (silent): print cmsg 
		except IOError, (errno):
			    if (silent): print "I/O error(%s): " % (errno)
			    result = False
		self.cif.canIdDelete(self.txid+node)
		cmsg = self.cmsg
		return result,cmsg
	def readNoRtr (self,node,cmsg=0,result=True,silent=False):
		self.cif.canIdAdd(self.txid+node)
		if (silent): print "PDO%dreadNoRtr node: %d -> " %(self.obj,node),
		#self.cmsg.canWriteByte(self.cif,(self.txid+node),0x10)    #rtr
		try: 
			self.cmsg.canRead(self.cif)
			if (silent): print cmsg 
		except IOError, (errno):
			    if (silent): print "I/O error(%s): " % (errno)
			    result = False
		self.cif.canIdDelete(self.txid+node)
		cmsg = self.cmsg
		return result,cmsg
		
		
		

