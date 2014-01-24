#!/bin/sh

echo "speedj([-0.1, 0, 0, 0, 0, 0],0.1,3)" | ncat 10.59.8.118 31001 --send-only
#echo "speedl([-0.5, 0, 0, 0, 0, 0],0.1,3)" | ncat 10.59.8.118 31001 --send-only
#sleep 1
#echo "stopl(0.1)" | ncat 10.59.8.118 31001 --send-only # Will make controller "Another thread is already controlling"
#echo "speedl([0, 0, 0, 0, 0, 0],-0.1,3)" | ncat 10.59.8.118 31001 --send-only



