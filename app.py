# Programa base para utilizar interfaz gráfica diseñada en QtDesigner

import sys
from PyQt5 import uic
from PyQt5.QtWidgets import QMainWindow, QApplication

import serial

class App(QMainWindow):
	def __init__(self):
		super().__init__()
		uic.loadUi("interfaz.ui", self)			# Ingresar nombre de su archivo .ui
		self.ser = serial.Serial(port="COM3", baudrate=9600, timeout=1.0)
		self.ser.close()
		self.pot1.sliderReleased.connect(self.get_value_servo1) #Devolver el dato cuando se suelta el slider
		self.pot2.sliderReleased.connect(self.get_value_servo2)
		self.pot3.sliderReleased.connect(self.get_value_servo3)
		self.pot4.sliderReleased.connect(self.get_value_servo4)

	def get_value_servo1(self): #funcion para el pot 1
		valor1 = self.pot1.value()
		dato1 = (valor1 & 252) #se hace esta opercion para que los dos primeros bits queden en 00
		print(chr(dato1))
		self.ser.open()
		self.ser.write( chr(int(dato1)).encode() ) #Se envia el dato del pot
		self.ser.close()
		self.label1.setText(str(valor1))#Se muestra el dato del pot en la etiqueta

	def get_value_servo2(self): #funcion para el pot 2
		valor2 = self.pot2.value()
		dato2 = ((valor2 | 1)& 253) #se hace esta opercion para que los dos primeros bits queden en 01
		print(chr(dato2))
		self.ser.open()
		self.ser.write( chr(int(dato2)).encode() )#Se envia el dato del pot
		self.ser.close()
		self.label2.setText(str(valor2))#Se muestra el dato del pot en la etiqueta

	def get_value_servo3(self): #funcion para el pot 3
		valor3 = self.pot3.value()
		dato3 = ((valor3 | 2)& 254)	#se hace esta opercion para que los dos primeros bits queden en 10
		print(chr(dato3))
		self.ser.open()
		self.ser.write( chr(int(dato3)).encode() )#Se envia el dato del pot
		self.ser.close()
		self.label3.setText(str(valor3))#Se muestra el dato del pot en la etiqueta

	def get_value_servo4(self): #funcion para el pot 4
		valor4 = self.pot4.value()
		dato4 = ((valor4 | 4)& 255)	#se hace esta opercion para que los dos primeros bits queden en 11
		print(chr(dato4))
		self.ser.open()
		self.ser.write(chr(dato4).encode() )#Se envia el dato del pot
		self.ser.close()
		self.label4.setText(str(valor4))#Se muestra el dato del pot en la etiqueta



if __name__ == '__main__':
	app = QApplication(sys.argv)
	GUI = App()
	GUI.show()
	sys.exit(app.exec_())
