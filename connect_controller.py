#!/usr/bin/python
# -*- coding: utf-8 -*-

from i611_MCS import *
from i611_extend import *
from rbsys import *
from i611_common import *
from i611_io import *
from i611shm import *
from threading import Event
import socket
import math

ADDR_IP = '192.168.1.23'
PORT    = 5000

def main():
    rb = i611Robot()
    _BASE = Base()
    
    try:
        rb.open()
    except:
        ans = input('[ZEUS] Turn on the Servo Plz (y/n)')
        if ans == 'y':
            rb.open()
        else:
            print('[ZEUS] Servo is Not On')
            return
    
    print('[ZEUS] Robot is Ready! ')
    IOinit(rb)
    
    m = MotionParam(jnt_speed=5, lin_speed=5, pose_speed=5, overlap=30)
    rb.motionparam(m)
    
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((ADDR_IP, PORT))
    print('[ZEUS] Connection is Now Starting ... ')
    srv.listen(5)
    
    try:
        while True:
            cli, addr = srv.accept()
            print('[ZEUS] Connection from {}'.format(addr))
            try:
                buf = u''
                while True:
                    chunk = cli.recv(4096)
                    if not chunk:
                        print('[ZEUS] Connection closed by peer') 
                        break
                    buf += chunk.decode('utf-8', 'replace')

                    while u'\n' in buf:
                        rb.asyncm(1)
                        
                        line, buf = buf.split(u'\n', 1)
                        line = line.strip()
                        if not line:
                            continue

                        # "cmd+payload" 형태 지원: 앞은 명령, 뒤는 인자 문자열
                        if u'+' in line:
                            cmd, payload = line.split(u'+', 1)
                            cmd = cmd.strip(); payload = payload.strip()
                        
                        # "cmd" 만 왔을 경우
                        else:
                            cmd, payload = line, u''

                        try:
                            if cmd == u'start':
                                cli.send('ready\n')
                                cli.send('done\n') # 어떠한 명령/오류 든 통신이 완료되면 'done' 전송
                                continue
                                
                            elif cmd == u'move_j_rel':
                                if not payload:
                                    cli.send('ERR:missing payload\n')
                                    cli.send('done\n')
                                    continue
                                try:
                                    vals = [float(x) for x in payload.split(',')]
                                    if len(vals) != 6:
                                        raise ValueError('need 6')
                                    rb.reljntmove(dj1=vals[0], dj2=vals[1], dj3=vals[2],
                                                dj4=vals[3], dj5=vals[4], dj6=vals[5])
                                    cli.send('ok\n')
                                    cli.send('done\n'); continue
                                except Exception as e:
                                    cli.send(('ERR:{0}\n'.format(e))); cli.send('done\n')
                                    continue

                            elif cmd == u'move_j_abs':
                                if not payload:
                                    cli.send('ERR:missing payload\n')
                                    cli.send('done\n')
                                    continue
                                try:
                                    vals = [float(x) for x in payload.split(',')]
                                    if len(vals) != 6:
                                        raise ValueError('need 6')
                                    J = Joint(vals[0], vals[1], vals[2], vals[3], vals[4], vals[5])
                                    rb.move(J)
                                    cli.send('ok\n')
                                    cli.send('done\n')
                                    continue
                                except Exception as e:
                                    cli.send(('ERR:{0}\n'.format(e))); cli.send('done\n')
                                    continue

                            elif cmd == u'move_l_rel':
                                if not payload:
                                    cli.send('ERR:missing payload\n')
                                    cli.send('done\n')
                                    continue
                                try:
                                    vals = [float(x) for x in payload.split(',')]
                                    if len(vals) != 6:
                                        raise ValueError('need 6')
                                    rb.relline(dx=vals[0], dy=vals[1], dz=vals[2], drz=vals[3], dry=vals[4], drx=vals[5])
                                    cli.send('ok\n')
                                    cli.send('done\n')
                                    continue
                                except Exception as e:
                                    cli.send(('ERR:{0}\n'.format(e))); cli.send('done\n')
                                    continue

                            elif cmd == u'move_l_abs':
                                if not payload:
                                    cli.send('ERR:missing payload\n')
                                    cli.send('done\n')
                                    continue
                                try:
                                    vals = [float(x) for x in payload.split(',')]
                                    if len(vals) not in (6, 7):
                                        raise ValueError('need 6 or 7 (posture)')
                                    if len(vals) == 7:
                                        P = Position(vals[0], vals[1], vals[2], vals[3], vals[4], vals[5], vals[6])
                                    else:
                                        P = Position(vals[0], vals[1], vals[2], vals[3], vals[4], vals[5])
                                    rb.move(P)
                                    cli.send('ok\n')
                                    cli.send('done\n')
                                    continue
                                except Exception as e:
                                    cli.send(('ERR:{0}\n'.format(e)))
                                    cli.send('done\n')
                                    continue
                                

                            elif cmd == u'joint_state':
                                joints = shm_read(0x3050, 6).split(',')
                                print('[ZEUS] joint : {}'.format(joints))
                                for i in range(6):
                                    joints[i] = round(math.degrees(float(joints[i])), 3)
                                resp = u','.join(unicode(j) for j in joints)
                                cli.send((resp + u'\n').encode('utf-8'))
                                cli.send('done\n')
                                continue

                            elif cmd == u'xy_state':
                                pose = shm_read(0x3000, 6).split(',')
                                print('[ZEUS] xy : {}'.format(pose))
                                pose[0] = round(float(pose[0])*1000, 3)
                                pose[1] = round(float(pose[1])*1000, 3)
                                pose[2] = round(float(pose[2])*1000, 3)
                                pose[3] = round(math.degrees(float(pose[3])), 3)
                                pose[4] = round(math.degrees(float(pose[4])), 3)
                                pose[5] = round(math.degrees(float(pose[5])), 3)
                                resp = u','.join(unicode(p) for p in pose)
                                cli.send((resp + u'\n').encode('utf-8'))
                                cli.send('done\n')
                                continue

                            else:
                                print('[ZEUS] Wrong Format:', repr(cmd))
                                cli.send('ERR\n')
                                cli.send('done\n')
                                continue

                        except Exception as e:
                            cli.send(('ERR:{0}\n'.format(e)))
                            cli.send('done\n')

            finally:
                try: cli.close()
                except: pass

    finally:
        try: 
            srv.close()
        except: 
            pass
        rb.asyncm(2)
        rb.close()

if __name__ == '__main__':
    main()