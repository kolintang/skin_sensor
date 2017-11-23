import sys, time, thread, msvcrt

global i
i = 0

def f():
    while True:
        global i
        raw_input('asdf;kajs;dlkfja;sldkjfa;lksjdf;lka')
        i = 1


thread.start_new_thread(f, ())

while True:

    global i
    print i
    i = i + 1
    time.sleep(1)

