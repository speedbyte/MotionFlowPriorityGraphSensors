
import socket
import sys
import time
import telephone
import e32
import appuifw


def run():
    def waiting_for_clients():
        global server_socket, client_socket, address, connected, socketout, heartbeat_start
        try:
            client_socket, address = server_socket.accept()
        except socket.error, err:
            print "accept error:", err
        client_socket.setblocking(False)
        connected = True
        heartbeat_start = time.time()
        print "connected with", address

    def disconnect_client():
        global client_socket, connected
        print "disconnect client..."
        connected = False
        client_socket.close()
        print "client disconnected"

    def recv_msg():
        global client_socket, app_signal, connected, heartbeat_start
        msg = ""
        try:
            msg = client_socket.recv(1024)
        except socket.error, err:
            print err
            connected = False
        if msg:
            heartbeat_start = time.time()
            eval_msg(msg)
        try:
            app_lock.signal()
        except: pass
        

    global server_socket, client_socket, connected, shutdown, timer, app_lock, heartbeat_timeout, heartbeat_start
    
    connected = False

    server_socket = socket.socket(socket.AF_BT,socket.SOCK_STREAM)
    server_socket.setblocking(False)
    port = socket.bt_rfcomm_get_available_server_channel(server_socket)
    server_socket.bind(("", port))
    server_socket.listen(5)
    socket.bt_advertise_service(u"Bluetooth Server", server_socket, True, socket.RFCOMM)
    socket.set_security(server_socket, socket.AUTHOR)
    
    while not shutdown:
        timer.cancel()
        if connected:
            #print "heartbeat_timeout;", heartbeat_timeout, ",", "time.time()-heartbeat_start", time.time()-heartbeat_start
            if (time.time()-heartbeat_start) > heartbeat_timeout:
                disconnect_client()
                continue
            
            timer.after(0.1, recv_msg)
            try:
                app_lock.wait()
            except: pass
        else:
            print "waiting for clients on port", port, "..."
            waiting_for_clients()


def send_msg(msg):
    global client_socket, connected
    print "send msg:", msg
    try:
        client_socket.sendall(msg)
    except socket.error, err:
        print err
        connected = False


def eval_msg(msg):
    global app_lock
    msg = msg.strip()
    print "eval:", repr(msg)
    
    command, param = msg.split(';')
    param = param.split(',')
    
    if command=="0x1C":
        send_msg(command+':%s' % "ok") 
        telephone.dial(param[0])
    elif command=="0x1D":
        telephone.hang_up()
        send_msg(command+':%s' % "ok")
    elif command=="1":
        pass
    else:
        send_msg(command+':%s' % "invalid command")






def quit():
    global server_socket, client_socket, shutdown, timer
    shutdown = True
    connected = False
    timer.cancel()
    server_socket.close()
    e32.ao_sleep(1)
    try:
        app_lock.signal()
    except: pass
    sys.exit(-1)


def start():
    try:
        app_lock.signal()
    except: pass
    global started
    if not started:
        try:
            started = True
            appuifw.note(u"server started", "info")
            run()
        except Exception, e:
            print e
    else:
        appuifw.note(u"server already started", "info")


def shutd():
    global started, shutdown, timer
    if started:
        try:
            shutdown = True
            started = False
            timer.cancel()
            e32.ao_sleep(1)
            try:
                app_lock.wait()
            except: pass
            appuifw.note(u"server stoped", "info")
        except Exception, e:
            print e
    else:
        appuifw.note(u"server is not started", "info")
        

heartbeat_timeout = 3.0
heartbeat_start = None
shutdown = connected = started = False
timer = e32.Ao_timer()
app_lock = e32.Ao_lock()

appuifw.app.exit_key_handler = quit
appuifw.app.title = u"Bluetooth Server"
appuifw.app.menu = [(u"start server", start), (u"shutdown server", shutd)]

app_lock.wait()


