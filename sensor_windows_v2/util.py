#-------------------------------------------------------------------------------
# esd gmbh  
#
#
#
#
#-------------------------------------------------------------------------------
# import pyQt			# spaeter vielleicht mal
import string
import sys
import re
import time


# functions
""" 

"""



def print2lines(): 
	print
	print
	
def print3lines(): 
	print
	print
	print

def print4lines(): 
	print
	print
	print

def print5lines(): 
	print
	print
	print

def l2c(l):
	d0=d1=d2=d3=0
	d0=(l>>24)&0xFF
	d1=(l>>16)&0xFF
	d2=(l>>8)&0xFF
	d3=(l>>0)&0xFF
	return d0,d1,d2,d3

def c2l(d0,d1,d2,d3):
	l = ((d0&0xFF)<<24)+((d1&0xFF)<<16)+((d2&0xFF)<<8)+((d3&0xFF))
	return l
	
	
def s2c(l):
	d0=d1=d2=d3=0
	d0=(l>>8)&0xFF
	d1=(l>>0)&0xFF
	return d0,d1,d2,d3

def c2s(d0,d1):
	s = ((d0&0xFF)<<8)+((d1&0xFF))
	return s
	

def psleep(t):
	print "Wait %fs ..."%t
	time.sleep(t)								

def sleep(t):
	time.sleep(t)								






def file_dict(name, sep="="): 
    f = file(name,'r') 
    c = f.read() 
    f.close() 
    regex = '((\w+)\s*%s[ \t\f\v]*(("[^"]*")|(\S*))[^\n]*\n)' % sep 
    result = dict([(x[1],x[2].strip('"').strip()) for x in re.findall(regex,c)]) 
    return result 
	 
"""
if len(sys.argv) == 2: 
    print file_to_dict(sys.argv[1],"[:=]")
"""



def base10toN(num,n):
    """Change a  to a base-n number.
    Up to base-36 is supported without special notation."""
    num_rep={10:'a',
         11:'b',
         12:'c',
         13:'d',
         14:'e',
         15:'f',
         16:'g',
         17:'h',
         18:'i',
         19:'j',
         20:'k',
         21:'l',
         22:'m',
         23:'n',
         24:'o',
         25:'p',
         26:'q',
         27:'r',
         28:'s',
         29:'t',
         30:'u',
         31:'v',
         32:'w',
         33:'x',
         34:'y',
         35:'z'}
    new_num_string=''
    current=num
    while current!=0:
        remainder=current%n
        if 36>remainder>9:
            remainder_string=num_rep[remainder]
        elif remainder>=36:
            remainder_string='('+str(remainder)+')'
        else:
            remainder_string=str(remainder)
        new_num_string=remainder_string+new_num_string
        current=current/n
    return new_num_string



def baseconvert(n, base):
    """convert positive decimal integer n to equivalent in another base (2-36)"""

    digits = "0123456789abcdefghijklmnopqrstuvwxyz"

    try:
        n = int(n)
        base = int(base)
    except:
        return ""
    
    if n < 0 or base < 2 or base > 36:
        return ""

    s = ""
    while 1:
        r = n % base
        s = digits[r] + s
        n = n / base
        if n == 0:
            break

    return s




def q(cond,on_true,on_false): 
        from inspect import isfunction 
 
        if cond: 
            if not isfunction(on_true): return on_true 
            else: return apply(on_true) 
        else: 
            if not isfunction(on_false): return on_false  
            else: return apply(on_false) 
    
