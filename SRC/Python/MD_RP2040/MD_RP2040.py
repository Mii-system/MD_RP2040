import minimalmodbus
import time

# Modbus機器の設定
PORT = '/dev/ttyACM0'  # シリアルポート
SLAVE_ADDRESS = 1      # Modbusデバイスのスレーブアドレス
REGISTER_ADDRESS = 1   # FC6で書き込むレジスタのアドレス

# Modbusインスタンスの初期化
modbus_device = minimalmodbus.Instrument(PORT, SLAVE_ADDRESS)
modbus_device.serial.baudrate = 9600  # ボーレート（デバイスに合わせて変更）
modbus_device.serial.bytesize = 8
modbus_device.serial.parity = minimalmodbus.serial.PARITY_NONE
modbus_device.serial.stopbits = 1
modbus_device.serial.timeout = 1  # タイムアウト（秒）

# サブルーチン: 指令値を送信する関数
def send_command(device, register, value):
    """
    Modbus機器に指令値を送信する
    :param device: minimalmodbus.Instrument オブジェクト
    :param register: レジスタのアドレス
    :param value: 送信する値（signed=Trueで正負の値を扱う）
    """
    print(f"Sending command: {value}")
    device.write_register(register, value, signed=True)
    time.sleep(1)  # 1秒待機

# 指令送信シーケンス
try:
    send_command(modbus_device, REGISTER_ADDRESS, 0)       # 0%
    send_command(modbus_device, REGISTER_ADDRESS, 2500)    # 25%
    send_command(modbus_device, REGISTER_ADDRESS, 5000)    # 50%
    send_command(modbus_device, REGISTER_ADDRESS, 2500)    # 25%
    send_command(modbus_device, REGISTER_ADDRESS, 0)       # 0%
    send_command(modbus_device, REGISTER_ADDRESS, -2500)   # -25%
    send_command(modbus_device, REGISTER_ADDRESS, -5000)   # -50%
    send_command(modbus_device, REGISTER_ADDRESS, -2500)   # -25%
    send_command(modbus_device, REGISTER_ADDRESS, 0)       # 0%
except Exception as e:
    print(f"エラーが発生しました: {e}")

print("終了しました。")
