#!/usr/bin/env python3
import time, threading, queue, os
from http.server import HTTPServer
from http.server import BaseHTTPRequestHandler
import urllib.parse as parse

from TemplateHandler import TemplateHandler
from BadRequestHandler import BadRequestHandler
from response.RequestHandler import StaticHandler

from GPIOSetup import ChassisQueue, ChassisQueuePreset




# WebServerQueue is used to control WebServer operation
# Queue message is an integer and defined as follows:
# 0 = Exit Thread
# 1 = Start web server
# 2 = Stop web server
WebServerQueue = queue.Queue()

def WebServerControllerThread():
    print("WebServerControllerThread")
    # Create an instance of the WebServer object
    webServer = WebServer("192.168.1.100", 8080)
    webServer.StartServerThread()

    while True:
        # Get instruction from the WebServerQueue
        webServerCmd = WebServerQueue.get()
        # Check WebServer command
        if webServerCmd == 1: # Start WebServer
            webServer.StartServer()
        elif webServerCmd == 2: # Stop WebServer
            webServer.StopServer()
        elif webServerCmd == 0: # Exit Thread
            webServer.StopServer()
            webServer.Set_isExiting(True)
            break
    
    # Wait for child thread to exit if exists
    if webServer.webServerThread != None:
        webServer.webServerThread.join()


class WebServer:
    # Class Constructor
    def __init__(self, host, port):
        self.host = host
        self.port = port
        self.isRunning = False
        self.isExiting = False
        self.webServerThread = None
        self.lock = threading.Lock()
        self.httpServer = HTTPServer((host, port), WebServerHandler)


    # Lock protected property accessors
    def Get_isRunning(self):
        self.lock.acquire()
        isRunning = self.isRunning
        self.lock.release()
        return isRunning

    def Set_isRunning(self, isRunning):
        self.lock.acquire()
        self.isRunning = isRunning
        self.lock.release()
   
    def Get_isExiting(self):
        self.lock.acquire()
        isExiting = self.isExiting
        self.lock.release()
        return isExiting

    def Set_isExiting(self, isExiting):
        self.lock.acquire()
        self.isExiting = isExiting
        self.lock.release()

    def StartServerThread(self):
        print("WebServer.StartServerThread()")
        if self.webServerThread == None:
            self.webServerThread = threading.Thread(target=WebServerThread, args=(self,))
            self.webServerThread.daemon = True
            self.webServerThread.start()

    def StartServer(self):
        print("WebServer.StartServer()")
        self.Set_isRunning(True)

    def StopServer(self):
        print("WebServer.StopServer()")
        if (self.webServerThread != None) & self.Get_isRunning():
            self.httpServer.shutdown()
            self.Set_isRunning(False)
            

routes = {
  "/" : {
    "template" : "index.html" 
  },
  "/empty" : {
    "template" : "empty.html"
  }
}

def WebServerThread(webServer):
    print("WebServerThread Started")

    # Loop while web server controller thread is active
    while webServer.Get_isExiting() == False:
        # Start Web server if in running state
        if webServer.Get_isRunning() == True:
            print(time.asctime(), 'Server UP - %s:%s' % (webServer.host, webServer.port))
            webServer.httpServer.serve_forever()
            print(time.asctime(), 'Server DOWN - %s:%s' % (webServer.host, webServer.port))
        time.sleep(1)

    # We are exiting, httpServer already shutdown    
    webServer.httpServer.server_close()
    

############################################################
# Start delivery run
############################################################
""" def StartDelivery():
    UltrasonicQueue.put(UltrasonicQueueAction.StartMeasurement)
    IRSensorQueue.put(IRSensorQueueAction.StartSensor)
    RFIDSensorQueue.put(RFIDSensorQueueAction.StartDetect) """



class WebServerHandler(BaseHTTPRequestHandler):
    def do_HEAD(self):
        return
    
    def do_GET(self):
        parse_result = parse.urlparse(self.path)
        query_str = parse.parse_qs(parse_result.query)
        split_path = os.path.splitext(parse_result.path)
        request_extension = split_path[1]
        handler = BadRequestHandler()

        # No extension or ".html"
        if request_extension == "" or request_extension == ".html":

            # Manual Chassis Control, return empty page. No page refresh
            if split_path[0] == "/manualctl":

                action = query_str.get("action")
                if type(action) == list:
                    action = int(action[0])
                """ if action == 0:
                    ChassisQueue.put(ChassisQueueAction.Stop)
                elif action == 1:
                    ChassisQueue.put(ChassisQueueAction.MoveForward)
                elif action == 2:
                    ChassisQueue.put(ChassisQueueAction.TurnLeft)
                elif action == 3:
                    ChassisQueue.put(ChassisQueueAction.TurnRight)
                elif action == 4:
                    ChassisQueue.put(ChassisQueueAction.SpinLeft90Deg)
                elif action == 5:
                    ChassisQueue.put(ChassisQueueAction.SpinRight90Deg)
                elif action == 6:
                    ChassisQueue.put(ChassisQueueAction.Reverse) """
                """ else:
                    ChassisQueue.put(ChassisQueueAction.Stop)
                handler = TemplateHandler()
                handler.find(routes["/empty"]) """

            # Delivery
            elif split_path[0] == "/delivery":
                action = query_str.get("action")
                """ if type(action) == list:
                    action = int(action[0])               
                # Add Delivery 
                if action == 1:
                    # Add Delivery
                    bin1 = int(query_str.get("bin1")[0])
                    pri1 = False if query_str.get("pri1")[0] == "0" else True
                    bin2 = int(query_str.get("bin2")[0])
                    pri2 = False if query_str.get("pri2")[0] == "0" else True
                    bin3 = int(query_str.get("bin3")[0])
                    pri3 = False if query_str.get("pri3")[0] == "0" else True
                
                    DELIVERYLIST.DeleteAll()
                    if bin1 != 0:
                        DELIVERYLIST.Add(RFIDCARDS.GetRoom(bin1), 1, pri1, RFIDCARDS.GetRoom(bin1, True))
                    if bin2 != 0:
                        DELIVERYLIST.Add(RFIDCARDS.GetRoom(bin2), 2, pri2, RFIDCARDS.GetRoom(bin2, True))
                    if bin3 != 0:
                        DELIVERYLIST.Add(RFIDCARDS.GetRoom(bin3), 3, pri3, RFIDCARDS.GetRoom(bin3, True))
                elif action == 2:
                    # Start delivery
                    StartDelivery()
                elif action == 3:
                    # Abort delivery
                    ChassisQueue.put(ChassisQueueAction.Stop)
                    IRSensorQueue.put(IRSensorQueueAction.StopSensor)
                    DELIVERYLIST.DeleteAll() """


            
            # Look for predefined route
            elif self.path in routes:
                handler = TemplateHandler()
                handler.find(routes[self.path])

            # Invalid .html request
            else:
                handler = BadRequestHandler()

        # Disallow .py
        elif request_extension == ".py":
            handler = BadRequestHandler()

        # Statie files
        else:
            handler = StaticHandler()
            handler.find(self.path)

        self.respond({
            "handler": handler
        })
    
    def do_POST(self):
        return
    
    #def handle_http(self, status, content_type):
    def handle_http(self, handler):
        status_code = handler.getStatus()

        self.send_response(status_code)

        if status_code == 200:
            content = handler.getContents()
            self.send_header('Content-type', handler.getContentType())
        else:
            content = "404 Not Found"

        self.end_headers()

        if isinstance(content, (bytes, bytearray)):
            return content

        return bytes(content, 'UTF-8')
    
    def respond(self, opts):
        response = self.handle_http(opts['handler'])
        self.wfile.write(response)