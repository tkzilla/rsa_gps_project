"""
RSA GPS Backend
This project connects to a USB RSA and USB GPS antenna
and requests a spectrum trace, latitude, longitude, 
altitude, and timestamp at a user-defined interval
and writes them to a csv file.
Tested using a Holux M-215+ USB GPS antenna and Tektronix RSA306B/RSA507A
Author: Morgan Allison
Date created: 3/17
Date edited: 6/17
Windows 7 64-bit
RSA_API version 3.9.0029
Python 3.6.0 64-bit (Anaconda 4.3.0)
NumPy 1.11.3, pyserial 3.2.1, pynmea2 1.7.1
Download Anaconda: http://continuum.io/downloads
NumPy included in Anaconda installation
pyserial: https://github.com/pyserial/pyserial (pip install pyserial)
pynmea2: https://github.com/Knio/pynmea2 (pip install pynmea2)
Download the RSA_API: http://www.tek.com/model/rsa306-software
Download the RSA_API Documentation:
http://www.tek.com/spectrum-analyzer/rsa306-manual-6

YOU WILL NEED TO REFERENCE THE API DOCUMENTATION
"""

from ctypes import *
from os import chdir
from enum import Enum
from time import perf_counter, strftime
import numpy as np
import threading, queue, serial, pynmea2, csv, sys

# C:\Tektronix\RSA_API\lib\x64 needs to be added to the 
# PATH system environment variable
chdir("C:\\Tektronix\\RSA_API\\lib\\x64")
rsa = cdll.LoadLibrary("RSA_API.dll")


class Spectrum_Settings(Structure):
    _fields_ = [('span', c_double), 
    ('rbw', c_double),
    ('enableVBW', c_bool), 
    ('vbw', c_double),
    ('traceLength', c_int), 
    ('window', c_int),
    ('verticalUnit', c_int), 
    ('actualStartFreq', c_double), 
    ('actualStopFreq', c_double),
    ('actualFreqStepSize', c_double), 
    ('actualRBW', c_double),
    ('actualVBW', c_double), 
    ('actualNumIQSamples', c_double)]


class Spectrum_TraceInfo(Structure):
    _fields_ = [('timestamp', c_int64), 
    ('acqDataStatus', c_uint16)]


class RSAError(Exception):
    pass

class GPSError(Exception):
    pass

class verticalUnits(Enum):
    dBm = 0
    W = 1
    Vrms = 2
    Arms = 3
    dBmV = 4
        

class GPS_Thread(threading.Thread):
    def __init__(self, threadID, name, gps, q):
        threading.Thread.__init__(self, daemon=True)
        self.threadID = threadID
        self.name = name
        self.gps = gps
        self.q = q
        self._stop = threading.Event()
        # self.dur = []

    def stop(self):
        self._stop.set()

    def stop_check(self):
        return self._stop.isSet()

    def run(self):
        while not self.stop_check():
            d = self.gps.readline()
            while not d.decode('cp1252').startswith('$GPGGA,') or len(d) > 80:
                if len(d) > 80:
                    print(d.decode('cp1252'))
                d = self.gps.readline()
            msg = pynmea2.parse(d.decode('cp1252'))
            if int(msg.num_sats) < 1:
                raise GPSError('No satellites locked.')
            # print('Thread duration: {}'.format(perf_counter()-t))
            # self.dur.append(perf_counter()-t)
            # print('Average thread duration: {}'.format(np.mean(self.dur)))
            self.q.put(msg)
        self.gps.close()


class Monitoring_Session:
    # Main class for project
    def __init__(self, startFreq, stopFreq, rbw, traceLength, verticalUnit, 
        refLevel, fileName, comPort, baudRate, timeInterval):
        self.startFreq = startFreq
        self.stopFreq = stopFreq
        self.rbw = rbw
        self.traceLength = traceLength
        self.verticalUnit = verticalUnit
        self.refLevel = refLevel
        self.fileName = fileName
        self.comPort = comPort
        self.baudRate = baudRate
        self.timeInterval = timeInterval
        self.specSet = Spectrum_Settings()
        self.traceInfo = Spectrum_TraceInfo()
        self.statusText = ''
        self.q = queue.LifoQueue()

        try:
            self.search_connect()
        except RSAError:
            raise
        except UnboundLocalError:
            pass
        finally:
            pass


    def search_connect(self):
        # Searches for and connects to an RSA device
        numFound = c_int(0)
        intArray = c_int*10
        deviceIDs = intArray()
        self.deviceSerial = create_string_buffer(8)
        self.deviceType = create_string_buffer(8)
        self.apiVersion = create_string_buffer(16)

        rsa.DEVICE_GetAPIVersion(self.apiVersion)
        # print('API Version {}'.format(apiVersion.value.decode()))

        ret = rsa.DEVICE_Search(byref(numFound), deviceIDs, 
            self.deviceSerial, self.deviceType)
        if ret != 0:
            raise RSAError('Error: {}'.format(ret))

        if numFound.value < 1:
            raise RSAError('Error: No instruments found.')
        elif numFound.value == 1:
            ret = -1
            while ret != 0:
                ret = rsa.DEVICE_Connect(deviceIDs[0])
                if ret != 0:
                    self.statusText = 'Error: {}.'.format(ret)
                    # raise RSAError(self.statusText)
                    rsa.DEVICE_Reset()
            self.statusText = 'Connected to {}, S/N {}.'.format(
                self.deviceType.value.decode(), 
                self.deviceSerial.value.decode())
            rsa.CONFIG_Preset()
        else:
            # corner case
            raise RSAError('Error: 2 or more instruments found.')


    def check_connect(self):
            ret = rsa.DEVICE_StartFrameTransfer()
            if ret != 0:
                raise RSAError('No RSA connected. Please connect to continue.')


    def convert_reference_level(self):
        # since CONFIG_SetReferenceLevel() argument is in dBm, convert 
        # from other units
        if self.verticalUnit == 'dBm':
            pass
        elif self.verticalUnit == 'W':
            self.refLevel = 10*np.log10(1000*self.refLevel)
        elif self.verticalUnit == 'Vrms':
            self.refLevel = 10*np.log10(self.refLevel**2/(50*1e-3))
        elif self.verticalUnit == 'Arms':
            self.refLevel = 10*np.log10(self.refLevel**2*50/1e-3)
        elif self.verticalUnit == 'dBmV':
            self.refLevel = self.refLevel - 10*np.log10(50) - 30
        else:
            # raise an exception here
            print('Invalid verticalUnit, unable to convert to dBm.')


    def output_file_header(self):
        # Writes the header for the output csv file
        try:
            with open(self.fileName, 'w') as csvfile:
                w = csv.writer(csvfile, lineterminator='\n')
                w.writerow(['Device Type:', self.deviceType.value.decode()])
                w.writerow(['Serial Number:', self.deviceSerial.value.decode()])
                w.writerow(['Start Freq:', self.specSet.actualStartFreq, 'Hz'])
                w.writerow(['Stop Freq:', self.specSet.actualStopFreq, 'Hz'])
                w.writerow(['RBW:', self.specSet.actualRBW, 'Hz'])
                w.writerow(['Vertical Units:', self.verticalUnit])
                w.writerow(['Reference Level:', self.refLevel, 'dBm'])
                w.writerow(['Trace Length:', self.specSet.traceLength])
                w.writerow(['Freq Step per Bin:', self.specSet.actualFreqStepSize, 
                    'Hz'])
                w.writerow(['Overrange', 'Date', 'Time', 'Latitude', 
                    'Longitude', 'Altitude', 'Alt Units', 'Trace Data'])
        except PermissionError as e:
            self.statusText = 'Permission error. Output .csv file is in use.'
            print(e)
            raise
        except FileNotFoundError:
            self.statusText = 'File not found.'
            raise
            

    def setup(self):
        # Sets spectrum settings on the RSA
        # start freq, stop freq, ref level, rbw, trace length, and units
        # also configures the GPS antenna or simulator
        self.startFreq *= 1e6
        self.stopFreq *= 1e6
        cf = np.mean([self.startFreq, self.stopFreq])
        span = float(np.diff([self.startFreq, self.stopFreq]))
        self.convert_reference_level()
        self.specSet = config_spectrum(cf, self.refLevel, span,
            self.rbw, self.traceLength, self.verticalUnit)
        try:
            self.output_file_header()
            self.gps_setup()
            self.gpsThread = GPS_Thread(0, 'gpsThread', self.gps, self.q)
            self.gpsThread.start()
        except PermissionError:
            raise


    def gps_setup(self):
        # tests device in selected COM port to see if it returns data
        try:
            self.gps = serial.Serial(self.comPort, baudrate=self.baudRate, 
                timeout=1)
            self.gps.flush()
            if not self.gps.readline():
                raise GPSError('{} returns no data.'.format(self.comPort))
        except PermissionError:
            raise


    def pause(self):
        while (perf_counter()-self.startTime) < self.timeInterval:
            self.q.get()


    def get_gga_sentence(self):
        # self.gps.flush()
        d = self.gps.readline()
        while not d.decode('cp1252').startswith('$GPGGA,') or len(d) > 80:
            if len(d) > 80:
                print(d.decode('cp1252'))
            d = self.gps.readline()
        msg = pynmea2.parse(d.decode('cp1252'))
        if int(msg.num_sats) < 1:
            raise GPSError('No satellites locked.')
        return msg


    def operation(self):
        # User-defined start/stop freq (MHz), RBW, trace points, vertical units,
        # reference level, COM port for GPS antenna, and acquisition time interval
        # verticalUnit: 0=dBm, 1=Watt, 2=Volt, 3=Amp, 4=dBmV
        # self.startTime = perf_counter()

        try:
            self.startTime = perf_counter()
            self.trace = acquire_spectrum(self.specSet)
            rsa.SPECTRUM_GetTraceInfo(byref(self.traceInfo))
            msg = self.q.get()
            print(msg)
        except pynmea2.nmea.ChecksumError:
            self.operation()
        except GPSError:
            raise
        # t2 = perf_counter()-self.startTime
        twoDigitTrace = list('{:.2f}'.format(point) for point in self.trace)
        try:
            with open(self.fileName, 'a') as csvfile:
                w = csv.writer(csvfile, lineterminator='\n')
                # overrange, date, time, lat, long, alt, trace data
                lat = '{:2.0f}\xb0{:2.0f}\'{:02.4f}"'.format(
                    msg.latitude, msg.latitude_minutes, msg.latitude_seconds)
                lon = '{:2.0f}\xb0{:2.0f}\'{:02.4f}"'.format(
                    msg.longitude, msg.longitude_minutes, msg.longitude_seconds)
                if (self.traceInfo.acqDataStatus & 0x1) == 0:
                    w.writerow([self.traceInfo.acqDataStatus, strftime('%x'), 
                        msg.timestamp, lat, lon, 
                        msg.altitude, msg.altitude_units, 
                        *twoDigitTrace])
                else:
                    w.writerow([self.traceInfo.acqDataStatus, strftime('%x'), 
                        msg.timestamp, lat, lon, 
                        msg.altitude, msg.altitude_units])
        except PermissionError:
            self.statusText = 'Permission error. Output .csv file is in use.'
            raise
        except FileNotFoundError:
            self.statusText = 'File not found.'
            raise
        # t3 = perf_counter()-self.startTime
        # print('Spectrum capture time: {}\nTime Remaining: {}'.format(t2, 
        #     self.timeInterval-t2))
        # print('Output time: {}\nTime Remaining: {}'.format(t3-t2, 
        #     self.timeInterval-t3))
        print(msg.timestamp)
        # self.pause()


def config_spectrum(cf=1e9, refLevel=0, span=40e6, rbw=300e3, 
    traceLength=801, verticalUnit=verticalUnits.dBm.name):
    # Configures spectrum settings on the RSA
    vUnit = {'dBm':0, 'W':1, 'Vrms':2, 'Arms':3, 'dBmV':4}[verticalUnit]

    rsa.SPECTRUM_SetEnable(c_bool(True))
    rsa.CONFIG_SetCenterFreq(c_double(cf))
    rsa.CONFIG_SetReferenceLevel(c_double(refLevel))
    rsa.SPECTRUM_SetDefault()
    specSet = Spectrum_Settings()
    rsa.SPECTRUM_GetSettings(byref(specSet))
    specSet.span = span
    specSet.rbw = rbw
    specSet.traceLength = traceLength
    specSet.verticalUnit = vUnit
    rsa.SPECTRUM_SetSettings(specSet)
    rsa.SPECTRUM_GetSettings(byref(specSet))
    return specSet


def create_frequency_array(specSet):
    # Create array of frequency data for plotting the spectrum.
    # Unused in demo script
    freq = np.arange(specSet.actualStartFreq, specSet.actualStartFreq 
        + specSet.actualFreqStepSize*specSet.traceLength, 
        specSet.actualFreqStepSize)
    return freq


def acquire_spectrum(specSet):
    # Acquires a single spectrum trace from the RSA
    ready = c_bool(False)
    traceArray = c_float*specSet.traceLength
    traceData = traceArray()
    outTracePoints = c_int(0)

    rsa.DEVICE_Run()
    rsa.SPECTRUM_AcquireTrace()    
    while ready.value == False:
        rsa.SPECTRUM_WaitForDataReady(c_int(100), byref(ready))
    rsa.SPECTRUM_GetTrace(c_int(0), specSet.traceLength, byref(traceData), 
        byref(outTracePoints))
    rsa.DEVICE_Stop()
    return np.array(traceData)


def list_serial_ports():
    # Lists serial port names
    if sys.platform.startswith('win'):
        ports = ['COM{}'.format(i+1) for i in range(256)]
    elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
        # this excludes your current terminal "/dev/tty"
        ports = glob.glob('/dev/tty[A-Za-z]*')
    elif sys.platform.startswith('darwin'):
        ports = glob.glob('/dev/tty.*')
    else:
        raise EnvironmentError('Unsupported platform.')

    result = []
    for port in ports:
        try:
            s = serial.Serial(port)
            s.close()
            result.append(port)
        except (OSError, serial.SerialException):
            pass
    return result


def main():
    # Demo script
    startFreq = 700
    stopFreq = 6200
    rbw = 10e6
    traceLength = 64001
    vUnit = verticalUnits.dBm.name
    refLevel = 0
    fileName = 'C:\\Users\\mallison\\Documents\\GitHub\\rsa_gps\\output.csv'
    comPort = 'COM12'   # COM12 for Holux GPS, COM3 for GPS simulator
    baudRate = 4800
    timeInterval = 3
    numTests = 5

    try:
        session = Monitoring_Session(startFreq, stopFreq, rbw, traceLength, 
            vUnit, refLevel, fileName, comPort, baudRate, timeInterval)
        print(session.statusText)
        session.setup()
        for i in range(numTests):
            # sleep(session.timeInterval)
            print('Capturing spectrum', i+1)
            session.operation()
            session.pause()
            # if i == 1:
                # session.gpsThread.stop()                
            # print(session.msg.timestamp)
    except (RSAError, GPSError, PermissionError, UnboundLocalError) as err:
        print(err)


if __name__ == '__main__':
    main()
