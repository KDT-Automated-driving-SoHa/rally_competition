# https://docs.python.org/ko/3/library/termios.html#module-termios
import sys
import os
import tty
import termios

from multiprocessing import Process, Queue

def process_keyboard_input(fd, queue):
    sys.stdin = os.fdopen(fd)

    attr = termios.tcgetattr(fd)
    attr[3] = attr[3] | termios.ECHO
    # termios.tcsetattr(fd, termios.TCSADRAIN, attr)

    while True:
        try:
            tty.setcbreak(fd)
            c = sys.stdin.read(1)
            if c != "":
                queue.put(c)
        except:
            termios.tcsetattr(fd, termios.TCSADRAIN, attr)
            break
        finally:
            pass


class KeyboardManager():
    def __init__(self):
        self.fd = sys.stdin.fileno()
        self.queue = Queue()

        self.process = Process(target=process_keyboard_input, args=(self.fd, self.queue))
        self.process.start()


    def get_keyboard_input(self):
        if not self.queue.empty():
            return True, self.queue.get_nowait()

        return False, None


    def release(self):
        self.process.join
    
        

if __name__ == '__main__':
    key_manager = KeyboardManager()

    while True:
        key = key_manager.get_keyboard_input()
        print("ret", key)
