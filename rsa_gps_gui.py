"""
RSA GPS GUI
This project connects to a USB RSA and USB GPS antenna
and requests a spectrum trace, latitude, longitude, 
altitude, and timestamp at a user-defined interval
and writes them to a csv file.
Tested using a Holux M-215+ USB GPS antenna and Tektronix RSA306B/RSA507A
Author: Morgan Allison
Date created: 3/17
Date edited: 6/17
Windows 7 64-bit
RSA API version 3.9.0029
Python 3.6.0 64-bit (Anaconda 4.3.0)
NumPy 1.11.3, MatPlotLib 2.0.0, PyQt5
Download Anaconda: http://continuum.io/downloads
Anaconda includes NumPy, MatPlotLib, and PyQt5
Download the RSA_API: http://www.tek.com/model/rsa306-software
Download the RSA_API Documentation:
http://www.tek.com/spectrum-analyzer/rsa306-manual-6

YOU WILL NEED TO REFERENCE THE API DOCUMENTATION
"""


import sys
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from rsa_gps_backend import *
from os import chdir


"""
################################################################
C:\Tektronix\RSA_API\lib\x64 needs to be added to the 
PATH system environment variable
################################################################
"""
chdir("C:\\Tektronix\\RSA_API\\lib\\x64")
rsa = cdll.LoadLibrary("RSA_API.dll")


"""#################CLASSES AND FUNCTIONS#################"""
class Window(QWidget):
    def __init__(self):
        # getting parent object
        super().__init__()
        self.setGeometry(50, 50, 400, 400)
        self.setWindowTitle('Session Setup')

        self.initUI()


    def initUI(self):
        # Monitoring session setup GUI
        self.msgDuration = 3000
        grid = QGridLayout(self)
        names = ['Start Freq (MHz):', 'Stop Freq (MHz):', 'RBW (Hz):', 
            'Ref Level:', 'Trace Length:', 'Vertical Units:', 
            'COM Port:', 'Baud Rate:', 'File Name:', 'Timer Interval (sec):']
        row = 0
        col = 0
        for name in names:
            label = QLabel(self)
            label.setText(name)
            grid.addWidget(label, row, col, Qt.AlignCenter)
            row += 1
        
        row += 1
        self.setupButton = QPushButton(self)
        self.setupButton.setText('Setup')
        self.setupButton.clicked.connect(self.setup)
        grid.addWidget(self.setupButton, row, col, Qt.AlignCenter)        
        
        col += 1
        self.startButton = QPushButton(self)
        self.startButton.setText('Start')
        self.startButton.setEnabled(False)
        self.startButton.clicked.connect(self.start)
        grid.addWidget(self.startButton, row, col, Qt.AlignCenter)

        row += 1
        self.stopButton = QPushButton(self)
        self.stopButton.setText('Stop')
        self.stopButton.setEnabled(False)
        self.stopButton.clicked.connect(self.stop)
        grid.addWidget(self.stopButton, row, col, Qt.AlignCenter)

        row += 1
        col = 0
        self.statusBar = QStatusBar(self)
        self.statusBar.setSizeGripEnabled(False)
        grid.addWidget(self.statusBar, row, col, -1, -1)

        row = 0
        col = 1
        self.startFInput = QLineEdit(self)
        self.startFInput.setValidator(QDoubleValidator())
        self.startFInput.setText('980')
        grid.addWidget(self.startFInput, row, col, Qt.AlignCenter)

        row += 1
        self.stopFInput = QLineEdit(self)
        self.stopFInput.setValidator(QDoubleValidator())
        self.stopFInput.setText('1020')
        grid.addWidget(self.stopFInput, row, col, Qt.AlignCenter)

        row += 1
        self.rbwInput = QLineEdit(self)
        self.rbwInput.setText('10000')
        self.rbwInput.setValidator(QDoubleValidator())
        grid.addWidget(self.rbwInput, row, col, Qt.AlignCenter)

        row += 1
        self.rlInput = QLineEdit(self)
        self.rlInput.setText('0')
        self.rbwInput.setValidator(QDoubleValidator())
        grid.addWidget(self.rlInput, row, col, Qt.AlignCenter)

        row += 1
        tLengthOptions = ['801', '2401', '4001', '8001', '10401', '16001', 
            '32001', '64001']
        self.tLengthInput = QComboBox(self)
        self.tLengthInput.addItems(tLengthOptions)
        grid.addWidget(self.tLengthInput, row, col, Qt.AlignCenter)

        row += 1
        unitOptions = ['dBm', 'W', 'Vrms', 'Arms', 'dBmV']
        self.unitInput = QComboBox(self)
        self.unitInput.addItems(unitOptions)
        grid.addWidget(self.unitInput, row, col, Qt.AlignCenter)

        row += 1
        portList = list_serial_ports()
        self.comPortInput = QComboBox(self)
        self.comPortInput.addItems(portList)
        self.comPortInput.setCurrentIndex(3)
        grid.addWidget(self.comPortInput, row, col, Qt.AlignCenter)
        
        row += 1
        baudOptions = ['4800', '9600', '14400', '19200', '38400', '57600', 
            '115200']
        self.baudInput = QComboBox(self)
        self.baudInput.addItems(baudOptions)
        grid.addWidget(self.baudInput, row, col, Qt.AlignCenter)
       
        row += 1
        self.fileNameInput = QLineEdit(self)
        self.fileNameInput.setText(
            'C:\\Users\\mallison\\Documents\\GitHub\\rsa_gps\\output.csv')
        grid.addWidget(self.fileNameInput, row, col, Qt.AlignCenter)

        row += 1
        self.timeIntInput = QLineEdit(self)
        self.timeIntInput.setText('1')
        self.timeIntInput.setValidator(QIntValidator())
        grid.addWidget(self.timeIntInput, row, col, Qt.AlignCenter)

        self.show()


    def setup(self):
        self.statusBar.showMessage('Setting up RSA...')

        self.acqTimer = QTimer()
        self.acqTimer.timeout.connect(self.operation)
        self.acqCounter = 1

        startFreq = float(self.startFInput.text())
        stopFreq = float(self.stopFInput.text())
        rbw = float(self.rbwInput.text())
        refLevel = float(self.rlInput.text())
        traceLength = int(self.tLengthInput.currentText())
        verticalUnit = self.unitInput.currentText()
        comPort = self.comPortInput.currentText()
        baudRate = int(self.baudInput.currentText())
        fileName = self.fileNameInput.text()
        timeInterval = float(self.timeIntInput.text())

        try:
            self.session = Monitoring_Session(startFreq, stopFreq, rbw, 
                traceLength, verticalUnit, refLevel, fileName, comPort, 
                baudRate, timeInterval)
            self.statusBar.showMessage(self.session.statusText)
            self.session.setup()
            self.startButton.setEnabled(True)
            self.activate_settings(False)

        except (RSAError, GPSError, AttributeError, PermissionError, 
            serial.serialutil.SerialException) as err:
            self.statusBar.showMessage(str(err), self.msgDuration)


    def operation(self):
        self.statusBar.showMessage('Capturing acquisition {}'.format(
            self.acqCounter), self.msgDuration)
        self.acqCounter += 1
        try:
            self.session.operation()
            # print(self.session.msg.timestamp)
            # print('Elapsed time: {}'.format(perf_counter()-t1))
            # print('Time interval: {}'.format(self.session.timeInterval))
        except AttributeError:
            raise
            self.statusBar.showMessage(
                'Run Setup before starting a capture session.')
        except GPSError:
            raise
            self.statusBar.showMessage(GPSError)


    def activate_settings(self, active):
        self.startFInput.setEnabled(active)
        self.stopFInput.setEnabled(active)
        self.rbwInput.setEnabled(active)
        self.rlInput.setEnabled(active)
        self.tLengthInput.setEnabled(active)
        self.unitInput.setEnabled(active)
        self.comPortInput.setEnabled(active)
        self.baudInput.setEnabled(active)
        self.fileNameInput.setEnabled(active)
        self.timeIntInput.setEnabled(active)
        self.setupButton.setEnabled(active)


    def start(self):
        self.operation()
        self.acqTimer.start(self.session.timeInterval*1e3)
        self.activate_settings(False)
        self.startButton.setEnabled(False)
        self.stopButton.setEnabled(True)


    def stop(self):
        self.statusBar.showMessage('Acquisition stopped', self.msgDuration)
        self.activate_settings(True)
        self.stopButton.setEnabled(False)
        # self.startButton.setEnabled(True)
        self.acqTimer.stop()
        self.session.gpsThread.stop()
        rsa.DEVICE_Stop()
        while not self.session.q.empty():
            self.session.q.get()


    def check_rsa_connection(self):
        try:
            self.session.check_connect()
        except RSAError as err:
            self.statusBar.showMessage(
                'No RSA connected. Connect to continue.')


def main():
    app = QApplication(sys.argv)
    GUI = Window()

    sys.exit(app.exec_())


if __name__ == "__main__":
    main()