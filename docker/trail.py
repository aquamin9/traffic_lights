#!/usr/bin/env python

class Foo(object):
    def __init__(self):

        self.Liste = [0,2,3,4]
        number = int(input("Zahl?"))

        if number in self.Liste : 
            self.Liste.remove(number)
        print(type(self.Liste))
        print(self.Liste)

a = Foo()

