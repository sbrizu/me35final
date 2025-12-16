import ubluetooth
from machine import Timer, sleep
import time
#import final_secrets
from encoder1 import Motor

_UART_SERVICE_UUID = ubluetooth.UUID("6E400001-B5A3-F393-E0A9-E50E24DCCA9E")
_UART_TX_CHAR_UUID = ubluetooth.UUID("6E400003-B5A3-F393-E0A9-E50E24DCCA9E")
_UART_RX_CHAR_UUID = ubluetooth.UUID("6E400002-B5A3-F393-E0A9-E50E24DCCA9E")

_IRQ_CENTRAL_CONNECT = 1
_IRQ_CENTRAL_DISCONNECT = 2
_IRQ_GATTS_WRITE = 3
LEFT_ADD = 5

# wheel1a, wheel1b, wheel2a, wheel2b, fly1a, fly1b, fly2a, fly2b):
#ourRobot = Robot(25, 26, 32, 33, 12, 13, 14, 27)



class BLEPeripheral:
    def _init_(self, name="ESP32test"):
        self.name = name
        self.ble = ubluetooth.BLE()
        self.ble.active(True)
        self.ble.irq(self.ble_irq)
        self.leftM = Motor(25,26)
        self.rightM = Motor(21,22)
        self.fly1 = Motor(14,27)
        self.fly2 = Motor(12,13)
        self.fly1.stop()
        self.fly2.stop()
        #self.bot.stopFlys()
        self.tx = (_UART_TX_CHAR_UUID, ubluetooth.FLAG_NOTIFY)
        self.rx = (_UART_RX_CHAR_UUID, ubluetooth.FLAG_WRITE)
        self.service = (_UART_SERVICE_UUID, (self.tx, self.rx))
        ((self.tx_handle, self.rx_handle),) = self.ble.gatts_register_services((self.service,))

        self.connections = set()
        time.sleep(0.5)
        self.advertise()

    def ble_irq(self, event, data):
        if event == _IRQ_CENTRAL_CONNECT:
            conn_handle, _, _ = data
            print("Central connected")
            self.connections.add(conn_handle)

        elif event == _IRQ_CENTRAL_DISCONNECT:
            conn_handle, _, _ = data
            print("Central disconnected")
            self.connections.discard(conn_handle)
            self.advertise()

        elif event == _IRQ_GATTS_WRITE:
            conn_handle, value_handle = data
            if value_handle == self.rx_handle:
                msg = self.ble.gatts_read(self.rx_handle)
                self.on_rx(msg)


    def on_rx(self, data):
        print("message recieved")
        print(data)
        if len(data) >= 4:
            l_dir, l_pow, r_dir, r_pow = data[:4]
            #self.bot.move(l_dir, l_pow, r_dir, r_pow)
            self.leftM.start(l_dir, l_pow + LEFT_ADD)
            self.rightM.start(r_dir, r_pow)
            print(f"L: dir={l_dir}, pow={l_pow} | R: dir={r_dir}, pow={r_pow}")

        elif len(data) >= 1:
            print("picking up")
            data = data[0]
            print(data)
            if data == 0:
                print("intake")
                self.fly1.start(0, 40)
                self.fly2.start(0, 40)
                #self.bot.ballIntake()
            else:
                print("shoot")
                self.fly1.start(1, 40)
                self.fly2.start(1, 40)
                #self.bot.ballShoot()
        else:
            print("Incomplete:", data)

    def send(self, msg):
        for conn in self.connections:
            self.ble.gatts_notify(conn, self.tx_handle, msg)

    def advertise(self):
        # Proper advertising with service UUID
        name_bytes = bytes(self.name, 'utf-8')
        service_bytes = bytes(_UART_SERVICE_UUID)
        adv_payload = bytearray([len(name_bytes)+1, 0x09]) + name_bytes
        adv_payload += bytearray([len(service_bytes)+1, 0x07]) + service_bytes
        self.ble.gap_advertise(100_000, adv_payload)
        print("Advertising as:", self.name)




# Usage
ble = BLEPeripheral()


'''
def heartbeat(timer):
    ble.send(b"ESP alive")

 timer= Timer(-1)
timer.init(period=1000, mode=Timer.PERIODIC, callback=heartbeat)
'''
while True:
    time.sleep(0.1)
